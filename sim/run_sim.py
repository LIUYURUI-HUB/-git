from __future__ import annotations

import argparse
import ctypes
import html
import json
import math
import struct
import subprocess
import sys
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Iterable
from urllib.parse import parse_qs, urlparse


REPO_ROOT = Path(__file__).resolve().parents[1]
SIM_DIR = Path(__file__).resolve().parent
DLL_PATH = SIM_DIR / "build" / "dm4340_sim.dll"
BUILD_SCRIPT = SIM_DIR / "build_sim.ps1"
OUTPUT_DIR = SIM_DIR / "output"
DEFAULT_HTML = OUTPUT_DIR / "vision_arm_simulation.html"
DEFAULT_BOX = (0.04, 0.0, 0.28)
HOME = (0.10, 0.0, -0.35)


class CSimSnapshot(ctypes.Structure):
    _fields_ = [
        ("tick_ms", ctypes.c_uint32),
        ("end_x", ctypes.c_float),
        ("end_y", ctypes.c_float),
        ("end_z", ctypes.c_float),
        ("pan_x", ctypes.c_float),
        ("pan_y", ctypes.c_float),
        ("pan_z", ctypes.c_float),
        ("target_x", ctypes.c_float),
        ("target_y", ctypes.c_float),
        ("target_z", ctypes.c_float),
        ("feedback_x", ctypes.c_float),
        ("feedback_y", ctypes.c_float),
        ("feedback_z", ctypes.c_float),
        ("state_code", ctypes.c_uint8),
        ("loaded_mode", ctypes.c_uint8),
        ("payload_mass", ctypes.c_float),
        ("feedback_count", ctypes.c_uint32),
        ("rx_frame_count", ctypes.c_uint32),
        ("motor_pos", ctypes.c_float * 4),
        ("motor_cmd", ctypes.c_float * 4),
    ]


@dataclass(frozen=True)
class Snapshot:
    tick_ms: int
    end: tuple[float, float, float]
    pan: tuple[float, float, float]
    target: tuple[float, float, float]
    feedback: tuple[float, float, float]
    state_code: int
    state_name: str
    loaded: bool
    payload_mass: float
    feedback_count: int
    rx_frame_count: int
    motor_pos: tuple[float, float, float, float]
    motor_cmd: tuple[float, float, float, float]


@dataclass(frozen=True)
class SimulationResult:
    detected_box: tuple[float, float, float]
    sent_frame: bytes
    trace: list[Snapshot]
    state_history: list[tuple[int, str]]
    target_consumed: bool
    observed_place: bool
    observed_retreat: bool
    final_position: tuple[float, float, float] | None
    home_error_m: float | None
    feedback_count: int


class SimLibrary:
    def __init__(self, dll_path: Path):
        self.dll_path = dll_path
        self._lib = ctypes.CDLL(str(dll_path))
        self._lib.sim_init.argtypes = []
        self._lib.sim_init.restype = None
        self._lib.sim_process_buffer.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32]
        self._lib.sim_process_buffer.restype = None
        self._lib.sim_step.argtypes = [ctypes.c_uint32]
        self._lib.sim_step.restype = None
        self._lib.sim_get_snapshot.argtypes = [ctypes.POINTER(CSimSnapshot)]
        self._lib.sim_get_snapshot.restype = None
        self._lib.sim_get_state_name.argtypes = []
        self._lib.sim_get_state_name.restype = ctypes.c_char_p

    @classmethod
    def load(cls, build_if_needed: bool = True) -> "SimLibrary":
        if build_if_needed and not DLL_PATH.exists():
            build_library()
        if not DLL_PATH.exists():
            raise FileNotFoundError(f"Missing simulator DLL: {DLL_PATH}")
        return cls(DLL_PATH)

    def init(self) -> None:
        self._lib.sim_init()

    def process_buffer(self, data: bytes) -> None:
        buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        self._lib.sim_process_buffer(buf, len(data))

    def step(self, dt_ms: int) -> None:
        self._lib.sim_step(dt_ms)

    def snapshot(self) -> Snapshot:
        raw = CSimSnapshot()
        self._lib.sim_get_snapshot(ctypes.byref(raw))
        raw_name = self._lib.sim_get_state_name()
        name = raw_name.decode("ascii", errors="replace") if raw_name else "UNKNOWN"
        return Snapshot(
            tick_ms=int(raw.tick_ms),
            end=(float(raw.end_x), float(raw.end_y), float(raw.end_z)),
            pan=(float(raw.pan_x), float(raw.pan_y), float(raw.pan_z)),
            target=(float(raw.target_x), float(raw.target_y), float(raw.target_z)),
            feedback=(float(raw.feedback_x), float(raw.feedback_y), float(raw.feedback_z)),
            state_code=int(raw.state_code),
            state_name=name,
            loaded=bool(raw.loaded_mode),
            payload_mass=float(raw.payload_mass),
            feedback_count=int(raw.feedback_count),
            rx_frame_count=int(raw.rx_frame_count),
            motor_pos=tuple(float(raw.motor_pos[i]) for i in range(4)),
            motor_cmd=tuple(float(raw.motor_cmd[i]) for i in range(4)),
        )


def build_library() -> None:
    subprocess.run(
        ["powershell", "-ExecutionPolicy", "Bypass", "-File", str(BUILD_SCRIPT)],
        cwd=REPO_ROOT,
        check=True,
    )


def pack_arm_target_frame(x: float, y: float, z: float) -> bytes:
    payload = struct.pack("<fff", x, y, z)
    frame = bytearray([0x55, 0xAA, 0x12, len(payload)])
    frame.extend(payload)
    frame.append(sum(frame) & 0xFF)
    return bytes(frame)


def _dist(a: Iterable[float], b: Iterable[float]) -> float:
    ax, ay, az = a
    bx, by, bz = b
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


def run_headless(
    max_ms: int = 22000,
    step_ms: int = 5,
    box: tuple[float, float, float] = DEFAULT_BOX,
    build_if_needed: bool = True,
) -> SimulationResult:
    lib = SimLibrary.load(build_if_needed=build_if_needed)
    lib.init()

    frame = pack_arm_target_frame(*box)
    lib.process_buffer(frame)

    trace: list[Snapshot] = []
    state_history: list[tuple[int, str]] = []
    target_consumed = False
    observed_place = False
    observed_retreat = False
    last_state = None
    final_snapshot: Snapshot | None = None

    for _ in range(0, max_ms, step_ms):
        lib.step(step_ms)
        snap = lib.snapshot()
        trace.append(snap)

        if snap.state_name != last_state:
            state_history.append((snap.tick_ms, snap.state_name))
            last_state = snap.state_name

        target_consumed = target_consumed or snap.rx_frame_count > 0 and snap.state_name != "WAIT_TARGET"
        observed_place = observed_place or snap.state_name == "PLACE"
        observed_retreat = observed_retreat or snap.state_name == "RETREAT"
        final_snapshot = snap

        if (
            observed_place
            and observed_retreat
            and snap.state_name == "WAIT_TARGET"
            and snap.tick_ms > 1000
            and _dist(snap.end, HOME) < 0.03
        ):
            break

    final_position = final_snapshot.end if final_snapshot else None
    home_error = _dist(final_position, HOME) if final_position is not None else None
    feedback_count = final_snapshot.feedback_count if final_snapshot else 0

    return SimulationResult(
        detected_box=box,
        sent_frame=frame,
        trace=trace,
        state_history=state_history,
        target_consumed=target_consumed,
        observed_place=observed_place,
        observed_retreat=observed_retreat,
        final_position=final_position,
        home_error_m=home_error,
        feedback_count=feedback_count,
    )


def _fmt_xyz(value: tuple[float, float, float] | None) -> str:
    if value is None:
        return "None"
    return f"x={value[0]: .3f} m, y={value[1]: .3f} m, z={value[2]: .3f} m"


def print_result(result: SimulationResult) -> None:
    print("DM4340 PC vision-arm simulation")
    print(f"Detected material box: {_fmt_xyz(result.detected_box)}")
    print(f"0x12 frame bytes: {result.sent_frame.hex(' ')}")
    print(f"Target consumed: {result.target_consumed}")
    print(f"Observed PLACE: {result.observed_place}")
    print(f"Observed RETREAT: {result.observed_retreat}")
    print(f"Final arm coordinate: {_fmt_xyz(result.final_position)}")
    if result.home_error_m is not None:
        print(f"Final distance to configured home: {result.home_error_m:.3f} m")
    print(f"Feedback frames captured: {result.feedback_count}")
    print("State history:")
    for tick, state in result.state_history:
        print(f"  {tick:5d} ms  {state}")


def simulation_payload(result: SimulationResult, sample_step: int = 4) -> dict:
    trace = result.trace[::max(1, sample_step)]
    if result.trace and (not trace or trace[-1].tick_ms != result.trace[-1].tick_ms):
        trace.append(result.trace[-1])

    return {
        "box": {
            "x": result.detected_box[0],
            "y": result.detected_box[1],
            "z": result.detected_box[2],
        },
        "frame": result.sent_frame.hex(" "),
        "targetConsumed": result.target_consumed,
        "observedPlace": result.observed_place,
        "observedRetreat": result.observed_retreat,
        "finalPosition": result.final_position,
        "homeError": result.home_error_m,
        "feedbackCount": result.feedback_count,
        "trace": [
            {
                "t": snap.tick_ms,
                "x": round(snap.end[0], 5),
                "y": round(snap.end[1], 5),
                "z": round(snap.end[2], 5),
                "tx": round(snap.target[0], 5),
                "ty": round(snap.target[1], 5),
                "tz": round(snap.target[2], 5),
                "state": snap.state_name,
                "loaded": snap.loaded,
            }
            for snap in trace
        ],
        "stateHistory": [{"t": tick, "state": state} for tick, state in result.state_history],
    }


def _write_showcase_html(output_path: Path, json_payload: str) -> Path:
    html_text = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>机械狗抓取与单侧绕背仿真</title>
  <style>
    *{box-sizing:border-box} body{margin:0;background:#111;color:#f5f7fb;font-family:"Segoe UI","Microsoft YaHei",Arial,sans-serif;overflow:hidden}
    #app{position:relative;width:100vw;height:100vh;min-height:720px;background:radial-gradient(circle at 72% 24%,rgba(50,80,105,.26),transparent 34%),linear-gradient(#1b1b1d,#101011)}
    #world{display:block;width:100%;height:100%}
    .panel{position:absolute;left:18px;top:24px;width:420px;padding:24px;background:rgba(18,18,24,.93);border:1px solid rgba(255,255,255,.13);border-radius:10px;box-shadow:0 24px 70px rgba(0,0,0,.38)}
    h1{margin:0 0 16px;color:#36f17c;font-size:22px;letter-spacing:0}.rule{height:1px;background:rgba(255,255,255,.13);margin-bottom:18px}
    .row{display:flex;align-items:center;justify-content:space-between;gap:14px;margin:13px 0;font-size:16px}.label{font-weight:700}.value{font-family:Consolas,monospace;color:#4da1ff;font-weight:700;text-align:right}
    .state{display:inline-flex;min-width:112px;justify-content:center;padding:6px 9px;border-radius:6px;background:#347cff;color:#fff;font-family:Consolas,monospace;font-weight:800}.note{margin-top:26px;color:#b5bdc9;font-size:13px;line-height:1.6}
    .controls{position:absolute;left:50%;bottom:34px;transform:translateX(-50%);display:flex;gap:16px}button{border:0;border-radius:9px;padding:16px 28px;color:#fff;font-weight:800;font-size:17px;background:linear-gradient(#377cff,#2764dc);box-shadow:0 0 28px rgba(55,124,255,.4);cursor:pointer}.secondary{background:#596274;box-shadow:0 0 22px rgba(89,98,116,.35)}
    .progress{position:absolute;left:470px;right:42px;top:28px;height:8px;border-radius:999px;background:rgba(255,255,255,.1);overflow:hidden}.progress span{display:block;height:100%;width:0;background:linear-gradient(90deg,#36f17c,#4da1ff)}
  </style>
</head>
<body>
<div id="app">
  <canvas id="world"></canvas>
  <div class="progress"><span id="bar"></span></div>
  <aside class="panel">
    <h1>🤖 视觉抓取自动任务状态机</h1><div class="rule"></div>
    <div class="row"><span class="label">当前状态:</span><span id="state" class="state">WAIT_TARGET</span></div>
    <div class="row"><span class="label">负载模式:</span><span id="load" class="value">空载（0.0kg）</span></div>
    <div style="height:16px"></div>
    <div class="row"><span class="label">坐标 X (高度):</span><span id="x" class="value">0.000</span></div>
    <div class="row"><span class="label">坐标 Y (侧偏):</span><span id="y" class="value">0.000</span></div>
    <div class="row"><span class="label">坐标 Z (前后):</span><span id="z" class="value">0.000</span></div>
    <div style="height:18px"></div>
    <div class="row"><span class="label">物资箱:</span><span id="box" class="value"></span></div>
    <div class="row"><span class="label">0x12:</span><span id="frame" class="value"></span></div>
    <p class="note">红色轨迹线代表末端真实运动路径；黄色物资箱会在吸取后跟随末端，放置后留在后背区域。动画数据来自本机 C 状态机仿真 trace。</p>
  </aside>
  <div class="controls"><button id="start">👀 视觉发现物资箱（开始仿真）</button><button id="reset" class="secondary">重置</button></div>
</div>
<script id="sim-data" type="application/json">__SIM_DATA__</script>
<script>
const data=JSON.parse(document.getElementById("sim-data").textContent),trace=data.trace;
const canvas=document.getElementById("world"),ctx=canvas.getContext("2d");
const stateEl=document.getElementById("state"),loadEl=document.getElementById("load"),xEl=document.getElementById("x"),yEl=document.getElementById("y"),zEl=document.getElementById("z"),boxEl=document.getElementById("box"),frameEl=document.getElementById("frame"),barEl=document.getElementById("bar"),startBtn=document.getElementById("start"),resetBtn=document.getElementById("reset");
let idx=0,running=true,last=performance.now(),acc=0;
const placeTime=(data.stateHistory.find(s=>s.state==="PLACE")||{t:Infinity}).t,placePos={x:0,y:0,z:.375};
function resize(){const dpr=Math.max(1,devicePixelRatio||1),r=canvas.getBoundingClientRect();canvas.width=Math.floor(r.width*dpr);canvas.height=Math.floor(r.height*dpr);ctx.setTransform(dpr,0,0,dpr,0,0)}
function project(p){const w=canvas.clientWidth,h=canvas.clientHeight,s=Math.min(w,h)*.86,cx=w*.58,cy=h*.62;return{x:cx+(p.z*.96-p.y*.70)*s,y:cy+(p.z*.30+p.y*.25-p.x*1.30)*s}}
function line(a,b,c,w){const pa=project(a),pb=project(b);ctx.strokeStyle=c;ctx.lineWidth=w;ctx.lineCap="round";ctx.beginPath();ctx.moveTo(pa.x,pa.y);ctx.lineTo(pb.x,pb.y);ctx.stroke()}
function cube(center,size,color){const p=project(center),s=size*Math.min(canvas.clientWidth,canvas.clientHeight)*.86;ctx.save();ctx.translate(p.x,p.y);ctx.fillStyle="rgba(0,0,0,.24)";ctx.beginPath();ctx.ellipse(0,s*.7,s*.75,s*.22,0,0,Math.PI*2);ctx.fill();ctx.fillStyle=color;ctx.fillRect(-s/2,-s/2,s,s);ctx.fillStyle="rgba(255,255,255,.35)";ctx.beginPath();ctx.moveTo(-s/2,-s/2);ctx.lineTo(-s/3,-s*.78);ctx.lineTo(s*.68,-s*.78);ctx.lineTo(s/2,-s/2);ctx.closePath();ctx.fill();ctx.strokeStyle="rgba(0,0,0,.28)";ctx.strokeRect(-s/2,-s/2,s,s);ctx.restore()}
function grid(){ctx.fillStyle="#151516";ctx.fillRect(0,0,canvas.clientWidth,canvas.clientHeight);for(let z=-.8;z<=.9;z+=.1)line({x:-.01,y:-.6,z},{x:-.01,y:.6,z},"rgba(255,255,255,.035)",1);for(let y=-.6;y<=.6;y+=.1)line({x:-.01,y,z:-.8},{x:-.01,y,z:.9},"rgba(255,255,255,.035)",1);line({x:0,y:0,z:-.8},{x:0,y:0,z:.9},"rgba(255,255,255,.16)",2);line({x:0,y:-.6,z:0},{x:0,y:.6,z:0},"rgba(255,255,255,.13)",2)}
function platform(){const cs=[{x:0,y:-.26,z:-.42},{x:0,y:.26,z:-.42},{x:0,y:.26,z:.44},{x:0,y:-.26,z:.44}].map(project);ctx.fillStyle="#3f3f3f";ctx.strokeStyle="#282828";ctx.beginPath();ctx.moveTo(cs[0].x,cs[0].y);for(let i=1;i<cs.length;i++)ctx.lineTo(cs[i].x,cs[i].y);ctx.closePath();ctx.fill();ctx.stroke();for(let z=-.4;z<=.44;z+=.1)line({x:.002,y:-.26,z},{x:.002,y:.26,z},"rgba(0,0,0,.23)",1);for(let y=-.2;y<=.25;y+=.1)line({x:.002,y,z:-.42},{x:.002,y,z:.44},"rgba(0,0,0,.23)",1);const back=[{x:.004,y:-.24,z:.2},{x:.004,y:.24,z:.2},{x:.004,y:.24,z:.44},{x:.004,y:-.24,z:.44}].map(project);ctx.fillStyle="rgba(42,167,88,.52)";ctx.beginPath();ctx.moveTo(back[0].x,back[0].y);for(let i=1;i<back.length;i++)ctx.lineTo(back[i].x,back[i].y);ctx.closePath();ctx.fill()}
function traceLine(){if(idx<2)return;ctx.strokeStyle="#ff4c4c";ctx.lineWidth=2;ctx.beginPath();for(let i=0;i<=idx;i++){const p=project(trace[i]);if(i===0)ctx.moveTo(p.x,p.y);else ctx.lineTo(p.x,p.y)}ctx.stroke()}
function arm(p){const base={x:0,y:0,z:0},shoulder={x:.055,y:0,z:0},end={x:p.x,y:p.y,z:p.z},elbow={x:Math.max(.15,(shoulder.x+end.x)*.5+.1),y:end.y*.42,z:end.z*.46};const b=project(base);ctx.fillStyle="#2e4b6a";ctx.beginPath();ctx.ellipse(b.x,b.y+14,30,12,0,0,Math.PI*2);ctx.fill();line(base,shoulder,"#6e7782",28);line(shoulder,elbow,"#d8d8d8",18);line(elbow,end,"#eee",16);line(shoulder,elbow,"rgba(255,255,255,.72)",8);line(elbow,end,"rgba(255,255,255,.72)",7);const pe=project(end);ctx.fillStyle=p.loaded?"#36f17c":"#ff4c4c";ctx.beginPath();ctx.arc(pe.x,pe.y,15,0,Math.PI*2);ctx.fill();ctx.strokeStyle="#1a1a1a";ctx.lineWidth=2;ctx.stroke()}
function boxPos(p){if(p.loaded)return{x:p.x,y:p.y,z:p.z};if(p.t>=placeTime)return placePos;return data.box}
function render(){const p=trace[idx];grid();platform();traceLine();cube(boxPos(p),.055,"#ffd91f");arm(p);stateEl.textContent=p.state;loadEl.textContent=p.loaded?"带载（0.5kg）":"空载（0.0kg）";xEl.textContent=p.x.toFixed(3);yEl.textContent=p.y.toFixed(3);zEl.textContent=p.z.toFixed(3);boxEl.textContent=`${data.box.x.toFixed(3)}, ${data.box.y.toFixed(3)}, ${data.box.z.toFixed(3)}`;frameEl.textContent=data.frame.slice(0,28)+" ...";barEl.style.width=(idx/Math.max(1,trace.length-1)*100).toFixed(2)+"%"}
function animate(now){const dt=now-last;last=now;if(running){acc+=dt;while(acc>18){acc-=18;idx++;if(idx>=trace.length){idx=trace.length-1;running=false;startBtn.textContent="播放完成（再次播放）";break}}render()}requestAnimationFrame(animate)}
startBtn.onclick=()=>{if(idx>=trace.length-1)idx=0;running=!running;startBtn.textContent=running?"暂停仿真":"👀 视觉发现物资箱（开始仿真）";render()};resetBtn.onclick=()=>{idx=0;running=false;startBtn.textContent="👀 视觉发现物资箱（开始仿真）";render()};addEventListener("resize",()=>{resize();render()});resize();render();requestAnimationFrame(animate);
</script>
</body>
</html>
""".replace("__SIM_DATA__", json_payload)
    output_path.write_text(html_text, encoding="utf-8")
    return output_path


def write_interactive_html() -> str:
    return """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>机械狗抓取与单侧绕背仿真 - 手动坐标</title>
  <style>
    *{box-sizing:border-box}body{margin:0;background:#101011;color:#f5f7fb;font-family:"Segoe UI","Microsoft YaHei",Arial,sans-serif;overflow:hidden}
    #app{position:relative;width:100vw;height:100vh;min-height:720px;background:radial-gradient(circle at 72% 24%,rgba(50,80,105,.28),transparent 34%),linear-gradient(#1b1b1d,#101011)}
    #world{display:block;width:100%;height:100%}.panel{position:absolute;left:18px;top:24px;width:436px;padding:24px;background:rgba(18,18,24,.94);border:1px solid rgba(255,255,255,.13);border-radius:10px;box-shadow:0 24px 70px rgba(0,0,0,.38)}
    h1{margin:0 0 16px;color:#36f17c;font-size:22px}.rule{height:1px;background:rgba(255,255,255,.13);margin-bottom:16px}.row{display:flex;align-items:center;justify-content:space-between;gap:14px;margin:11px 0;font-size:15px}.label{font-weight:700}.value{font-family:Consolas,monospace;color:#4da1ff;font-weight:700;text-align:right}.state{display:inline-flex;min-width:112px;justify-content:center;padding:6px 9px;border-radius:6px;background:#347cff;color:#fff;font-family:Consolas,monospace;font-weight:800}
    .inputs{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin:14px 0 10px}.inputs label{font-size:12px;color:#b5bdc9}.inputs input{width:100%;margin-top:5px;padding:8px 9px;border:1px solid rgba(255,255,255,.18);border-radius:7px;background:#0e1118;color:#fff;font-family:Consolas,monospace}
    .apply{width:100%;margin:4px 0 12px;padding:11px;border:0;border-radius:8px;background:linear-gradient(#37d979,#219d54);color:#06120b;font-weight:900;cursor:pointer}.note{margin-top:18px;color:#b5bdc9;font-size:13px;line-height:1.55}
    .controls{position:absolute;left:50%;bottom:34px;transform:translateX(-50%);display:flex;gap:16px}button{border:0;border-radius:9px;padding:16px 28px;color:#fff;font-weight:800;font-size:17px;background:linear-gradient(#377cff,#2764dc);box-shadow:0 0 28px rgba(55,124,255,.4);cursor:pointer}.secondary{background:#596274;box-shadow:0 0 22px rgba(89,98,116,.35)}.progress{position:absolute;left:486px;right:42px;top:28px;height:8px;border-radius:999px;background:rgba(255,255,255,.1);overflow:hidden}.progress span{display:block;height:100%;width:0;background:linear-gradient(90deg,#36f17c,#4da1ff)}
  </style>
</head>
<body>
<div id="app">
  <canvas id="world"></canvas><div class="progress"><span id="bar"></span></div>
  <aside class="panel">
    <h1>🤖 视觉抓取自动任务状态机</h1><div class="rule"></div>
    <div class="inputs">
      <label>X 高度(m)<input id="box-x" type="number" step="0.01" value="0.04"></label>
      <label>Y 侧偏(m)<input id="box-y" type="number" step="0.01" value="0.00"></label>
      <label>Z 前后(m)<input id="box-z" type="number" step="0.01" value="0.28"></label>
    </div>
    <button id="apply" class="apply">应用坐标并重生成仿真</button>
    <div class="row"><span class="label">当前状态:</span><span id="state" class="state">WAIT_TARGET</span></div>
    <div class="row"><span class="label">负载模式:</span><span id="load" class="value">空载（0.0kg）</span></div>
    <div style="height:10px"></div>
    <div class="row"><span class="label">坐标 X (高度):</span><span id="x" class="value">0.000</span></div>
    <div class="row"><span class="label">坐标 Y (侧偏):</span><span id="y" class="value">0.000</span></div>
    <div class="row"><span class="label">坐标 Z (前后):</span><span id="z" class="value">0.000</span></div>
    <div class="row"><span class="label">0x12:</span><span id="frame" class="value"></span></div>
    <div class="row"><span class="label">结果:</span><span id="checks" class="value"></span></div>
    <p class="note">输入物资箱坐标后，页面会请求本机 Python 后台重新运行 C 状态机仿真，再播放新的连续轨迹。</p>
  </aside>
  <div class="controls"><button id="start">👀 视觉发现物资箱（开始仿真）</button><button id="reset" class="secondary">重置</button></div>
</div>
<script>
let data=null,trace=[],idx=0,running=false,last=performance.now(),acc=0,placeTime=Infinity,placePos={x:0,y:0,z:.375};
const canvas=document.getElementById("world"),ctx=canvas.getContext("2d"),stateEl=document.getElementById("state"),loadEl=document.getElementById("load"),xEl=document.getElementById("x"),yEl=document.getElementById("y"),zEl=document.getElementById("z"),frameEl=document.getElementById("frame"),checksEl=document.getElementById("checks"),barEl=document.getElementById("bar"),startBtn=document.getElementById("start");
function resize(){const dpr=Math.max(1,devicePixelRatio||1),r=canvas.getBoundingClientRect();canvas.width=Math.floor(r.width*dpr);canvas.height=Math.floor(r.height*dpr);ctx.setTransform(dpr,0,0,dpr,0,0)}
function project(p){const w=canvas.clientWidth,h=canvas.clientHeight,s=Math.min(w,h)*.86,cx=w*.58,cy=h*.62;return{x:cx+(p.z*.96-p.y*.70)*s,y:cy+(p.z*.30+p.y*.25-p.x*1.30)*s}}
function line(a,b,c,w){const pa=project(a),pb=project(b);ctx.strokeStyle=c;ctx.lineWidth=w;ctx.lineCap="round";ctx.beginPath();ctx.moveTo(pa.x,pa.y);ctx.lineTo(pb.x,pb.y);ctx.stroke()}
function cube(c,size,color){const p=project(c),s=size*Math.min(canvas.clientWidth,canvas.clientHeight)*.86;ctx.save();ctx.translate(p.x,p.y);ctx.fillStyle="rgba(0,0,0,.24)";ctx.beginPath();ctx.ellipse(0,s*.7,s*.75,s*.22,0,0,Math.PI*2);ctx.fill();ctx.fillStyle=color;ctx.fillRect(-s/2,-s/2,s,s);ctx.fillStyle="rgba(255,255,255,.35)";ctx.beginPath();ctx.moveTo(-s/2,-s/2);ctx.lineTo(-s/3,-s*.78);ctx.lineTo(s*.68,-s*.78);ctx.lineTo(s/2,-s/2);ctx.closePath();ctx.fill();ctx.strokeStyle="rgba(0,0,0,.28)";ctx.strokeRect(-s/2,-s/2,s,s);ctx.restore()}
function grid(){ctx.fillStyle="#151516";ctx.fillRect(0,0,canvas.clientWidth,canvas.clientHeight);for(let z=-.8;z<=.9;z+=.1)line({x:-.01,y:-.6,z},{x:-.01,y:.6,z},"rgba(255,255,255,.035)",1);for(let y=-.6;y<=.6;y+=.1)line({x:-.01,y,z:-.8},{x:-.01,y,z:.9},"rgba(255,255,255,.035)",1);line({x:0,y:0,z:-.8},{x:0,y:0,z:.9},"rgba(255,255,255,.16)",2);line({x:0,y:-.6,z:0},{x:0,y:.6,z:0},"rgba(255,255,255,.13)",2)}
function platform(){const cs=[{x:0,y:-.26,z:-.42},{x:0,y:.26,z:-.42},{x:0,y:.26,z:.44},{x:0,y:-.26,z:.44}].map(project);ctx.fillStyle="#3f3f3f";ctx.beginPath();ctx.moveTo(cs[0].x,cs[0].y);for(let i=1;i<cs.length;i++)ctx.lineTo(cs[i].x,cs[i].y);ctx.closePath();ctx.fill();for(let z=-.4;z<=.44;z+=.1)line({x:.002,y:-.26,z},{x:.002,y:.26,z},"rgba(0,0,0,.23)",1);const back=[{x:.004,y:-.24,z:.2},{x:.004,y:.24,z:.2},{x:.004,y:.24,z:.44},{x:.004,y:-.24,z:.44}].map(project);ctx.fillStyle="rgba(42,167,88,.52)";ctx.beginPath();ctx.moveTo(back[0].x,back[0].y);for(let i=1;i<back.length;i++)ctx.lineTo(back[i].x,back[i].y);ctx.closePath();ctx.fill()}
function traceLine(){if(idx<2)return;ctx.strokeStyle="#ff4c4c";ctx.lineWidth=2;ctx.beginPath();for(let i=0;i<=idx;i++){const p=project(trace[i]);if(i===0)ctx.moveTo(p.x,p.y);else ctx.lineTo(p.x,p.y)}ctx.stroke()}
function arm(p){const base={x:0,y:0,z:0},shoulder={x:.055,y:0,z:0},end={x:p.x,y:p.y,z:p.z},elbow={x:Math.max(.15,(shoulder.x+end.x)*.5+.1),y:end.y*.42,z:end.z*.46};const b=project(base);ctx.fillStyle="#2e4b6a";ctx.beginPath();ctx.ellipse(b.x,b.y+14,30,12,0,0,Math.PI*2);ctx.fill();line(base,shoulder,"#6e7782",28);line(shoulder,elbow,"#d8d8d8",18);line(elbow,end,"#eee",16);line(shoulder,elbow,"rgba(255,255,255,.72)",8);line(elbow,end,"rgba(255,255,255,.72)",7);const pe=project(end);ctx.fillStyle=p.loaded?"#36f17c":"#ff4c4c";ctx.beginPath();ctx.arc(pe.x,pe.y,15,0,Math.PI*2);ctx.fill()}
function boxPos(p){if(!data)return{x:.04,y:0,z:.28};if(p.loaded)return{x:p.x,y:p.y,z:p.z};if(p.t>=placeTime)return placePos;return data.box}
function render(){grid();platform();if(!trace.length){cube({x:.04,y:0,z:.28},.055,"#ffd91f");return}const p=trace[idx];traceLine();cube(boxPos(p),.055,"#ffd91f");arm(p);stateEl.textContent=p.state;loadEl.textContent=p.loaded?"带载（0.5kg）":"空载（0.0kg）";xEl.textContent=p.x.toFixed(3);yEl.textContent=p.y.toFixed(3);zEl.textContent=p.z.toFixed(3);frameEl.textContent=data.frame.slice(0,28)+" ...";checksEl.textContent=`PLACE=${data.observedPlace}, RETREAT=${data.observedRetreat}`;barEl.style.width=(idx/Math.max(1,trace.length-1)*100).toFixed(2)+"%"}
async function loadSim(){const x=document.getElementById("box-x").value,y=document.getElementById("box-y").value,z=document.getElementById("box-z").value;checksEl.textContent="正在运行 C 状态机...";const res=await fetch(`/simulate?x=${encodeURIComponent(x)}&y=${encodeURIComponent(y)}&z=${encodeURIComponent(z)}`);data=await res.json();trace=data.trace;idx=0;running=true;placeTime=(data.stateHistory.find(s=>s.state==="PLACE")||{t:Infinity}).t;startBtn.textContent="暂停仿真";render()}
function animate(now){const dt=now-last;last=now;if(running&&trace.length){acc+=dt;while(acc>18){acc-=18;idx++;if(idx>=trace.length){idx=trace.length-1;running=false;startBtn.textContent="播放完成（再次播放）";break}}render()}requestAnimationFrame(animate)}
document.getElementById("apply").onclick=loadSim;startBtn.onclick=()=>{if(idx>=trace.length-1)idx=0;running=!running;startBtn.textContent=running?"暂停仿真":"👀 视觉发现物资箱（开始仿真）";render()};document.getElementById("reset").onclick=()=>{idx=0;running=false;startBtn.textContent="👀 视觉发现物资箱（开始仿真）";render()};addEventListener("resize",()=>{resize();render()});resize();render();loadSim();requestAnimationFrame(animate);
</script>
</body>
</html>"""


def run_server(host: str = "127.0.0.1", port: int = 8765) -> None:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/":
                body = write_interactive_html().encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if parsed.path == "/simulate":
                query = parse_qs(parsed.query)
                try:
                    x = float(query.get("x", [DEFAULT_BOX[0]])[0])
                    y = float(query.get("y", [DEFAULT_BOX[1]])[0])
                    z = float(query.get("z", [DEFAULT_BOX[2]])[0])
                    result = run_headless(max_ms=26000, box=(x, y, z), build_if_needed=True)
                    body = json.dumps(simulation_payload(result), ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except Exception as exc:
                    body = json.dumps({"error": str(exc)}, ensure_ascii=False).encode("utf-8")
                    self.send_response(500)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                return
            self.send_response(404)
            self.end_headers()

        def log_message(self, format: str, *args) -> None:
            return

    build_library()
    server = ThreadingHTTPServer((host, port), Handler)
    print(f"Manual coordinate simulation page: http://{host}:{port}/")
    server.serve_forever()

def write_html_report(result: SimulationResult, output_path: Path = DEFAULT_HTML) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    points = [
        {
            "t": snap.tick_ms,
            "x": round(snap.end[0], 5),
            "y": round(snap.end[1], 5),
            "z": round(snap.end[2], 5),
            "tx": round(snap.target[0], 5),
            "ty": round(snap.target[1], 5),
            "tz": round(snap.target[2], 5),
            "state": snap.state_name,
            "loaded": snap.loaded,
        }
        for snap in result.trace[::4]
    ]
    state_history = [{"t": tick, "state": state} for tick, state in result.state_history]
    payload = {
        "box": {
            "x": result.detected_box[0],
            "y": result.detected_box[1],
            "z": result.detected_box[2],
        },
        "frame": result.sent_frame.hex(" "),
        "targetConsumed": result.target_consumed,
        "observedPlace": result.observed_place,
        "observedRetreat": result.observed_retreat,
        "finalPosition": result.final_position,
        "homeError": result.home_error_m,
        "feedbackCount": result.feedback_count,
        "trace": points,
        "stateHistory": state_history,
    }
    json_payload = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    return _write_showcase_html(output_path, json_payload)
    title = "DM4340 视觉取放 1:1 状态机仿真"
    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(title)}</title>
  <style>
    :root {{
      color-scheme: light;
      --ink: #171717;
      --muted: #66645f;
      --line: #d8d6cd;
      --paper: #f7f4ed;
      --panel: #fffefa;
      --blue: #1f70c8;
      --green: #20855d;
      --orange: #c47522;
      --violet: #7b4ac9;
      --red: #c34036;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Segoe UI", "Microsoft YaHei", Arial, sans-serif;
      background: var(--paper);
      color: var(--ink);
    }}
    header {{
      padding: 18px 24px 12px;
      border-bottom: 1px solid var(--line);
      background: #fffefa;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 24px;
      letter-spacing: 0;
    }}
    .sub {{
      margin: 0;
      color: var(--muted);
      font-size: 14px;
    }}
    main {{
      display: grid;
      grid-template-columns: minmax(620px, 1fr) 420px;
      gap: 18px;
      padding: 18px 24px 24px;
    }}
    section {{
      min-width: 0;
    }}
    .stage, .side {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
    }}
    .stage {{
      padding: 12px;
    }}
    .side {{
      padding: 16px;
    }}
    canvas {{
      display: block;
      width: 100%;
      height: auto;
      background: #fbfaf5;
      border-radius: 6px;
    }}
    .controls {{
      display: flex;
      align-items: center;
      gap: 10px;
      margin-top: 12px;
      flex-wrap: wrap;
    }}
    button {{
      border: 1px solid #272727;
      background: #fdfcf7;
      color: var(--ink);
      border-radius: 6px;
      padding: 8px 12px;
      font-size: 14px;
      cursor: pointer;
    }}
    button:hover {{
      background: #f0eee5;
    }}
    input[type="range"] {{
      flex: 1;
      min-width: 220px;
    }}
    .metric {{
      display: grid;
      grid-template-columns: 130px 1fr;
      gap: 8px;
      padding: 7px 0;
      border-bottom: 1px solid #ece9df;
      font-size: 14px;
    }}
    .metric b {{
      font-weight: 600;
    }}
    .metric span {{
      font-family: Consolas, monospace;
      overflow-wrap: anywhere;
    }}
    h2 {{
      margin: 0 0 12px;
      font-size: 18px;
    }}
    .history {{
      margin-top: 16px;
      max-height: 270px;
      overflow: auto;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #fbfaf5;
    }}
    .history div {{
      display: grid;
      grid-template-columns: 90px 1fr;
      gap: 8px;
      padding: 6px 8px;
      border-bottom: 1px solid #ece9df;
      font-family: Consolas, monospace;
      font-size: 13px;
    }}
    .history div:last-child {{ border-bottom: 0; }}
    .ok {{
      color: #145c32;
      font-weight: 700;
    }}
    .warn {{
      color: var(--red);
      font-weight: 700;
    }}
    @media (max-width: 980px) {{
      main {{
        grid-template-columns: 1fr;
      }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>{html.escape(title)}</h1>
    <p class="sub">这个页面播放的是电脑端 C 仿真库跑出的真实状态机 trace：视觉发现物资箱，按 0x12 协议传坐标，状态机连续执行取放。</p>
  </header>
  <main>
    <section class="stage">
      <canvas id="scene" width="980" height="660"></canvas>
      <div class="controls">
        <button id="play">暂停</button>
        <button id="restart">重播</button>
        <label>速度 <select id="speed">
          <option value="0.5">0.5x</option>
          <option value="1" selected>1x</option>
          <option value="2">2x</option>
          <option value="4">4x</option>
        </select></label>
        <input id="scrub" type="range" min="0" max="0" value="0">
      </div>
    </section>
    <aside class="side">
      <h2>仿真结果</h2>
      <div class="metric"><b>状态</b><span id="state"></span></div>
      <div class="metric"><b>时间</b><span id="time"></span></div>
      <div class="metric"><b>物资箱坐标</b><span id="box"></span></div>
      <div class="metric"><b>当前目标</b><span id="target"></span></div>
      <div class="metric"><b>末端坐标</b><span id="end"></span></div>
      <div class="metric"><b>吸取负载</b><span id="loaded"></span></div>
      <div class="metric"><b>0x12 帧</b><span id="frame"></span></div>
      <div class="metric"><b>完整性</b><span id="checks"></span></div>
      <div class="metric"><b>最终坐标</b><span id="final"></span></div>
      <div class="metric"><b>回 home 误差</b><span id="home"></span></div>
      <h2 style="margin-top:18px">状态机时间线</h2>
      <div id="history" class="history"></div>
    </aside>
  </main>
  <script id="sim-data" type="application/json">{json_payload}</script>
  <script>
    const data = JSON.parse(document.getElementById("sim-data").textContent);
    const trace = data.trace;
    const canvas = document.getElementById("scene");
    const ctx = canvas.getContext("2d");
    const scrub = document.getElementById("scrub");
    const playBtn = document.getElementById("play");
    const restartBtn = document.getElementById("restart");
    const speedSel = document.getElementById("speed");
    let index = 0;
    let playing = true;
    let lastFrameTime = performance.now();
    let accumulator = 0;
    scrub.max = Math.max(0, trace.length - 1);

    function xyz(v) {{
      if (!v) return "None";
      return `x=${{v[0].toFixed(3)}} m, y=${{v[1].toFixed(3)}} m, z=${{v[2].toFixed(3)}} m`;
    }}

    function pointText(p) {{
      return `x=${{p.x.toFixed(3)}} m, y=${{p.y.toFixed(3)}} m, z=${{p.z.toFixed(3)}} m`;
    }}

    function mapSide(p) {{
      const left = 54, top = 72, right = 604, bottom = 314;
      return [
        left + (p.z + 0.45) / 0.90 * (right - left),
        bottom - p.x / 0.38 * (bottom - top),
      ];
    }}

    function mapTop(p) {{
      const left = 54, top = 390, right = 604, bottom = 616;
      return [
        left + (p.z + 0.45) / 0.90 * (right - left),
        bottom - (p.y + 0.25) / 0.50 * (bottom - top),
      ];
    }}

    function drawGrid(bounds, title, xLabel, yLabel) {{
      const [left, top, right, bottom] = bounds;
      ctx.strokeStyle = "#242424";
      ctx.lineWidth = 2;
      ctx.strokeRect(left, top, right - left, bottom - top);
      ctx.strokeStyle = "#dedbd1";
      ctx.lineWidth = 1;
      for (let i = 1; i < 5; i++) {{
        const x = left + i * (right - left) / 5;
        const y = top + i * (bottom - top) / 5;
        ctx.beginPath();
        ctx.moveTo(x, top);
        ctx.lineTo(x, bottom);
        ctx.moveTo(left, y);
        ctx.lineTo(right, y);
        ctx.stroke();
      }}
      ctx.fillStyle = "#171717";
      ctx.font = "700 18px Segoe UI";
      ctx.fillText(title, left, top - 18);
      ctx.fillStyle = "#66645f";
      ctx.font = "13px Segoe UI";
      ctx.fillText(yLabel, left, top + 18);
      ctx.fillText(xLabel, right - 55, bottom + 22);
    }}

    function drawTrace(mapper, color) {{
      if (index < 2) return;
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i <= index; i++) {{
        const p = trace[i];
        const [x, y] = mapper(p);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }}
      ctx.stroke();
    }}

    function drawMarker(x, y, color, label) {{
      ctx.fillStyle = color;
      ctx.strokeStyle = "#171717";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(x, y, 7, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = "#171717";
      ctx.font = "13px Segoe UI";
      ctx.fillText(label, x + 11, y - 8);
    }}

    function drawArm(mapper, p, loaded) {{
      const base = mapper({{x: 0, y: 0, z: 0}});
      const end = mapper(p);
      ctx.strokeStyle = loaded ? "#20855d" : "#1f70c8";
      ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.moveTo(base[0], base[1]);
      ctx.lineTo(end[0], end[1]);
      ctx.stroke();
      drawMarker(base[0], base[1], "#171717", "base");
      drawMarker(end[0], end[1], loaded ? "#20855d" : "#1f70c8", "end");
    }}

    function render() {{
      const p = trace[index];
      const box = data.box;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#fbfaf5";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      drawGrid([54, 72, 604, 314], "侧视图：Z 前后 / X 高度", "Z", "X");
      drawGrid([54, 390, 604, 616], "俯视图：Z 前后 / Y 侧向", "Z", "Y");
      drawTrace(mapSide, "#2b70c9");
      drawTrace(mapTop, "#2b70c9");
      drawArm(mapSide, p, p.loaded);
      drawArm(mapTop, p, p.loaded);
      const boxSide = mapSide(box);
      const boxTop = mapTop(box);
      const targetSide = mapSide({{x: p.tx, y: p.ty, z: p.tz}});
      const targetTop = mapTop({{x: p.tx, y: p.ty, z: p.tz}});
      drawMarker(boxSide[0], boxSide[1], "#c47522", "box");
      drawMarker(boxTop[0], boxTop[1], "#c47522", "box");
      drawMarker(targetSide[0], targetSide[1], "#7b4ac9", "target");
      drawMarker(targetTop[0], targetTop[1], "#7b4ac9", "target");
      ctx.fillStyle = "#171717";
      ctx.font = "700 18px Segoe UI";
      ctx.fillText("实时状态机：" + p.state, 650, 96);
      ctx.font = "14px Consolas";
      ctx.fillText("蓝线为 C 状态机驱动出的末端连续轨迹", 650, 124);
      ctx.fillText("绿色表示吸盘负载已置位", 650, 148);

      document.getElementById("state").textContent = p.state;
      document.getElementById("time").textContent = (p.t / 1000).toFixed(2) + " s";
      document.getElementById("box").textContent = pointText(box);
      document.getElementById("target").textContent = `x=${{p.tx.toFixed(3)}} m, y=${{p.ty.toFixed(3)}} m, z=${{p.tz.toFixed(3)}} m`;
      document.getElementById("end").textContent = pointText(p);
      document.getElementById("loaded").textContent = p.loaded ? "yes" : "no";
      document.getElementById("frame").textContent = data.frame;
      document.getElementById("checks").innerHTML =
        `<span class="${{data.targetConsumed && data.observedPlace && data.observedRetreat ? "ok" : "warn"}}">` +
        `target=${{data.targetConsumed}}, PLACE=${{data.observedPlace}}, RETREAT=${{data.observedRetreat}}</span>`;
      document.getElementById("final").textContent = xyz(data.finalPosition);
      document.getElementById("home").textContent = data.homeError === null ? "None" : data.homeError.toFixed(3) + " m";
      scrub.value = index;
    }}

    function buildHistory() {{
      const node = document.getElementById("history");
      node.innerHTML = "";
      for (const item of data.stateHistory) {{
        const row = document.createElement("div");
        row.innerHTML = `<span>${{item.t}} ms</span><span>${{item.state}}</span>`;
        node.appendChild(row);
      }}
    }}

    function animate(now) {{
      const elapsed = now - lastFrameTime;
      lastFrameTime = now;
      if (playing && trace.length > 1) {{
        accumulator += elapsed * Number(speedSel.value);
        while (accumulator >= 20) {{
          accumulator -= 20;
          index += 4;
          if (index >= trace.length) {{
            index = trace.length - 1;
            playing = false;
            playBtn.textContent = "播放";
            break;
          }}
        }}
        render();
      }}
      requestAnimationFrame(animate);
    }}

    playBtn.addEventListener("click", () => {{
      playing = !playing;
      playBtn.textContent = playing ? "暂停" : "播放";
      if (playing && index >= trace.length - 1) index = 0;
    }});
    restartBtn.addEventListener("click", () => {{
      index = 0;
      playing = true;
      playBtn.textContent = "暂停";
      render();
    }});
    scrub.addEventListener("input", () => {{
      index = Number(scrub.value);
      playing = false;
      playBtn.textContent = "播放";
      render();
    }});

    buildHistory();
    render();
    requestAnimationFrame(animate);
  </script>
</body>
</html>
"""
    output_path.write_text(html_text, encoding="utf-8")
    return output_path


def run_gui(box: tuple[float, float, float] = DEFAULT_BOX, step_ms: int = 5) -> None:
    try:
        import tkinter as tk
    except ImportError as exc:
        raise RuntimeError("tkinter is not available on this Python installation") from exc

    lib = SimLibrary.load(build_if_needed=True)
    lib.init()
    frame = pack_arm_target_frame(*box)
    lib.process_buffer(frame)

    root = tk.Tk()
    root.title("DM4340 PC Vision Arm Simulation")
    canvas = tk.Canvas(root, width=1120, height=720, bg="#f7f7f3", highlightthickness=0)
    canvas.pack(fill="both", expand=True)

    trace: list[Snapshot] = []
    state_history: list[tuple[int, str]] = []
    last_state = {"name": None}
    done = {"value": False}

    side = (40, 70, 540, 330)
    top = (40, 390, 540, 650)
    panel = (590, 70, 1080, 650)

    def map_side(pos: tuple[float, float, float]) -> tuple[float, float]:
        x, _y, z = pos
        left, top_y, right, bottom = side
        sx = left + (z + 0.45) / 0.90 * (right - left)
        sy = bottom - (x / 0.36) * (bottom - top_y)
        return sx, sy

    def map_top(pos: tuple[float, float, float]) -> tuple[float, float]:
        _x, y, z = pos
        left, top_y, right, bottom = top
        sx = left + (z + 0.45) / 0.90 * (right - left)
        sy = bottom - (y + 0.25) / 0.50 * (bottom - top_y)
        return sx, sy

    def draw_grid(bounds: tuple[int, int, int, int], title: str, x_label: str, y_label: str) -> None:
        left, top_y, right, bottom = bounds
        canvas.create_rectangle(left, top_y, right, bottom, outline="#272727", width=2)
        for i in range(1, 5):
            x = left + i * (right - left) / 5
            y = top_y + i * (bottom - top_y) / 5
            canvas.create_line(x, top_y, x, bottom, fill="#deded8")
            canvas.create_line(left, y, right, y, fill="#deded8")
        canvas.create_text(left, top_y - 24, anchor="w", text=title, fill="#1b1b1b", font=("Segoe UI", 14, "bold"))
        canvas.create_text(right, bottom + 22, anchor="e", text=x_label, fill="#4c4c4c", font=("Segoe UI", 10))
        canvas.create_text(left, top_y - 6, anchor="w", text=y_label, fill="#4c4c4c", font=("Segoe UI", 10))

    def draw_trace(points: list[tuple[float, float]], color: str) -> None:
        if len(points) < 2:
            return
        flat = []
        for point in points:
            flat.extend(point)
        canvas.create_line(*flat, fill=color, width=2, smooth=True)

    def draw_marker(point: tuple[float, float], color: str, label: str) -> None:
        px, py = point
        canvas.create_oval(px - 6, py - 6, px + 6, py + 6, fill=color, outline="#1b1b1b")
        canvas.create_text(px + 10, py - 10, anchor="w", text=label, fill="#1b1b1b", font=("Segoe UI", 9))

    def redraw() -> None:
        canvas.delete("all")
        snap = trace[-1] if trace else lib.snapshot()

        draw_grid(side, "Side view: Z forward/back vs X height", "Z axis", "X height")
        draw_grid(top, "Top view: Z forward/back vs Y side", "Z axis", "Y side")
        canvas.create_rectangle(*panel, outline="#272727", width=2)

        side_points = [map_side(s.end) for s in trace]
        top_points = [map_top(s.end) for s in trace]
        draw_trace(side_points, "#2b70c9")
        draw_trace(top_points, "#2b70c9")

        draw_marker(map_side(box), "#d48b28", "box")
        draw_marker(map_top(box), "#d48b28", "box")
        draw_marker(map_side(snap.target), "#8a4bd6", "target")
        draw_marker(map_top(snap.target), "#8a4bd6", "target")
        draw_marker(map_side(snap.end), "#1a9b60" if snap.loaded else "#1f78d1", "end")
        draw_marker(map_top(snap.end), "#1a9b60" if snap.loaded else "#1f78d1", "end")

        canvas.create_text(610, 95, anchor="w", text="Simulation telemetry", fill="#1b1b1b", font=("Segoe UI", 16, "bold"))
        status_lines = [
            f"time: {snap.tick_ms / 1000.0:5.2f} s",
            f"state: {snap.state_name}",
            f"loaded: {'yes' if snap.loaded else 'no'}",
            f"detected box: {_fmt_xyz(box)}",
            f"current target: {_fmt_xyz(snap.target)}",
            f"end effector: {_fmt_xyz(snap.end)}",
            f"feedback frames: {snap.feedback_count}",
            f"0x12 frame: {frame.hex(' ')}",
        ]
        y = 132
        for line in status_lines:
            canvas.create_text(610, y, anchor="w", text=line, fill="#242424", font=("Consolas", 11))
            y += 25

        canvas.create_text(610, y + 10, anchor="w", text="State history", fill="#1b1b1b", font=("Segoe UI", 13, "bold"))
        y += 40
        for tick, state in state_history[-13:]:
            canvas.create_text(610, y, anchor="w", text=f"{tick:5d} ms  {state}", fill="#242424", font=("Consolas", 11))
            y += 23

        if done["value"]:
            canvas.create_text(
                610,
                620,
                anchor="w",
                text="Workflow reached PLACE and RETREAT. Headless output prints final coordinates.",
                fill="#145c32",
                font=("Segoe UI", 11, "bold"),
            )

    def tick() -> None:
        for _ in range(4):
            lib.step(step_ms)
        snap = lib.snapshot()
        trace.append(snap)

        if snap.state_name != last_state["name"]:
            state_history.append((snap.tick_ms, snap.state_name))
            last_state["name"] = snap.state_name

        if any(state == "PLACE" for _, state in state_history) and any(state == "RETREAT" for _, state in state_history):
            done["value"] = done["value"] or snap.state_name == "WAIT_TARGET"

        redraw()
        if not done["value"] and snap.tick_ms < 26000:
            root.after(20, tick)

    tick()
    root.mainloop()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DM4340 PC-only vision and arm simulation")
    parser.add_argument("--headless", action="store_true", help="run without opening the Tk visual window")
    parser.add_argument("--html", action="store_true", help="write a static HTML animation page from the C simulation trace")
    parser.add_argument("--serve", action="store_true", help="start a local browser page with manual material-box coordinate inputs")
    parser.add_argument("--host", default="127.0.0.1", help="host for --serve")
    parser.add_argument("--port", type=int, default=8765, help="port for --serve")
    parser.add_argument("--html-path", type=Path, default=DEFAULT_HTML, help="HTML output path")
    parser.add_argument("--box-x", type=float, default=DEFAULT_BOX[0], help="detected box x/height coordinate")
    parser.add_argument("--box-y", type=float, default=DEFAULT_BOX[1], help="detected box y/side coordinate")
    parser.add_argument("--box-z", type=float, default=DEFAULT_BOX[2], help="detected box z/forward coordinate")
    parser.add_argument("--max-ms", type=int, default=22000, help="maximum simulated milliseconds for headless mode")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    box = (args.box_x, args.box_y, args.box_z)

    if args.serve:
        run_server(args.host, args.port)
        return 0

    if args.html:
        result = run_headless(max_ms=args.max_ms, box=box, build_if_needed=True)
        output_path = write_html_report(result, args.html_path)
        print_result(result)
        print(f"HTML page: {output_path}")
        return 0 if result.target_consumed and result.observed_place and result.observed_retreat else 1

    if args.headless:
        result = run_headless(max_ms=args.max_ms, box=box, build_if_needed=True)
        print_result(result)
        return 0 if result.target_consumed and result.observed_place and result.observed_retreat else 1

    run_gui(box=box)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

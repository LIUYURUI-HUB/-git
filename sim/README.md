# DM4340 PC Vision Arm Simulation

This simulator runs on the PC only. It builds a host C DLL from the existing
firmware protocol parser, vision arm state machine, kinematics, and gravity
helpers, then uses Python to drive and visualize the process.

## Build

```powershell
powershell -ExecutionPolicy Bypass -File sim\build_sim.ps1
```

## Headless Verification

```powershell
python -m unittest sim.tests.test_headless -v
python sim\run_sim.py --headless --max-ms 26000
```

The default simulated material box is:

```text
x=0.040 m, y=0.000 m, z=0.280 m
```

## Static Visual Page

```powershell
python sim\run_sim.py --html --max-ms 26000
```

The generated page is:

```text
sim\output\vision_arm_simulation.html
```

Open that file in a browser to see the continuous process. The trace shown on
the page is generated from the host C simulation, not hand-authored animation.

## Live Tk Window

```powershell
python sim\run_sim.py
```

This opens a live Tk canvas if the local Python installation has `tkinter`.


# PC Vision Arm Simulation Design

## Goal

Build a PC-only closed-loop simulation for the existing DM4340 arm workflow. The simulation must show a continuous, visible process: a vision scene finds a material box, sends the box coordinate through the same arm-target protocol used by firmware, and the existing arm state machine performs hover, descend, suction wait, lift, move, place, and retreat.

The simulator is not a replacement controller. It is a host-side visual verification tool whose output should match the firmware path closely enough that the observed simulation sequence predicts the real run sequence.

## Existing Interfaces

- Arm target input uses protocol frame `[0x55][0xAA][0x12][12][float x][float y][float z][checksum]`.
- Arm end-position feedback uses `Protocol_SendArmPositionFeedback(x, y, z)`.
- The automatic workflow lives in `Task_Vision_State_Machine()`.
- Cartesian arm movement and IK/FK live in `Algorithm/kinematic/arm.c`.
- Current firmware main loop has the vision state machine commented out and drag-teach mode enabled; the PC simulator will not depend on the firmware main loop.

## Recommended Architecture

Create a new `sim/` directory with a host-compiled C simulation core and a Python visual runner.

The C core will reuse the firmware code where practical:

- `Task/arm_task.c`
- `Task/protocol_handler.c`
- `Algorithm/kinematic/arm.c`
- `Algorithm/dynamics/arm_g.c`

Host adapter files will provide desktop replacements for hardware-only functions and globals:

- `HAL_GetTick()` returns simulated milliseconds.
- `CDC_Transmit_HS()` captures feedback frames instead of using USB.
- `DM_Send_Ctrl()` captures desired motor commands.
- Simulated motor positions move toward captured motor commands at a bounded rate.
- Firmware global arm parameters, motor structs, and end-effector state are defined for the host build.

Python will load the host C core with `ctypes`, create the visual scene, generate the material-box detection, send the protocol frame into `Protocol_ProcessBuffer()`, step the state machine at a fixed interval, and draw the continuous motion.

## Visual Runner

The Python window will show:

- A top-down/side simplified arm workspace.
- The material box and its detected coordinate.
- The current target coordinate sent to firmware logic.
- The arm end-effector trace as a continuous path.
- The current state-machine stage.
- A timeline/log of target sent, suction wait, lift, move to back, place, and retreat.
- A final placement result showing where the box was picked and where it was placed.

The first version will use only Python standard library UI primitives where possible so it runs on this machine without extra installs.

## Data Flow

1. Python creates a material box in workspace coordinates.
2. The vision simulation detects the box center and converts it into the existing arm coordinate convention: `x` height, `y` side offset, `z` forward/back.
3. Python packs a `0x12` frame and calls the C core's protocol buffer function.
4. The C protocol parser sets the new arm target flag.
5. `Task_Vision_State_Machine()` consumes the target and drives the workflow.
6. `run_arm_to_pos()` computes motor commands via the existing IK path.
7. The host adapter moves simulated motor positions toward commands.
8. `run_arm_kinematics()` updates the end-effector coordinate.
9. Python renders the arm position, target, trace, current state, and final result.

## Scope

In scope:

- PC-only run path.
- Continuous visible simulation.
- Host C build that reuses the current state machine, protocol parser, IK/FK, and gravity mode flags.
- A repeatable demo scenario with a material box and automatic pick/place.
- A verification command that runs a headless simulation and checks that the state machine reaches placement and returns home.

Out of scope:

- Real camera integration.
- Real USB serial communication.
- Real motor/CAN timing fidelity.
- Physics-accurate suction contact dynamics.
- Modifying the production firmware main loop behavior.

## Error Handling

- If the target is unreachable, the visual runner shows the failure and the headless verification exits non-zero.
- If the C shared library is missing, Python reports the build command to run.
- If feedback frames are malformed, the simulation log reports protocol decode failure.

## Testing

- Build the host C core with MinGW GCC.
- Run a headless Python simulation to completion.
- Verify that a `0x12` target is consumed.
- Verify that state progression reaches place and retreat/home.
- Verify that output includes a clear final pick coordinate and final place coordinate.


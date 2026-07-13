# lite3_quadruped — Lite3 quadruped chassis primitive

Wraps the Lite3 quadruped robot motion host and exposes posture control
and burst-style movement to pilot via two MCP tools. Use this capability
whenever the user asks the robot to stand, sit, initialize, or move
(forward/backward/rotate).

## Interface (2 MCP tools)

### `robonix/primitive/chassis/posture` — set_posture

Set the Lite3 quadruped to a named posture. This is the **only way** to
transition between postures; there is no automatic posture change on move.

| field | type | meaning |
|-------|------|---------|
| `posture_name` | string | one of `"stand"`, `"sit"`, `"remote"` |

Returns `{success, message}`. `success=true` means the UDP command was
dispatched to the motion host.

**Available posture names:**

| posture_name | UDP command | description |
|-------------|-------------|-------------|
| `"stand"` | `0x21010202` | standing posture, ready for movement |
| `"sit"` | `0x21010202` | sitting posture (**same UDP command as stand**) |
| `"remote"` | `0x21010C02` | enter remote-control mode |

> **IMPORTANT:** `"stand"` and `"sit"` use the **same underlying UDP
> command** (`0x21010202`). The pilot **must track the robot's current
> posture as state** to avoid redundant calls. If the robot is already
> standing, calling `"stand"` again is a no-op (do not send it).

### `robonix/primitive/chassis/move` — move

Send a single-shot Twist command to the robot for a short duration,
then stop. **Not a continuous stream** — each call is a discrete burst.

| field | type | meaning |
|-------|------|---------|
| `command.forward_m` | float (meters) | move forward (positive) or backward (negative) |
| `command.rotate_deg` | float (degrees) | rotate in place (positive = CCW) |
| `command.linear_x` | float (m/s) | raw linear X velocity (if `forward_m` is 0) |
| `command.linear_y` | float (m/s) | raw linear Y velocity |
| `command.linear_z` | float (m/s) | raw linear Z velocity |
| `command.angular_x` | float (rad/s) | raw angular X velocity |
| `command.angular_y` | float (rad/s) | raw angular Y velocity |
| `command.angular_z` | float (rad/s) | raw angular Z velocity |
| `command.duration_sec` | float | duration for raw velocity mode (default: 1.0 s) |

**Movement modes (priority order):**

1. If `forward_m ≠ 0`: moves at `speed_mps` (default 0.2 m/s) for the
   calculated duration. Duration = `abs(forward_m) / speed_mps`.
2. Else if `rotate_deg ≠ 0`: rotates in place using `/leg_odom2` yaw as
   closed-loop feedback. It slows near the target, stops within the configured
   tolerance, and reports the measured angle or an odometry/timeout error.
3. Else: raw velocity mode using `linear_x/y/z`, `angular_x/y/z` for
   `duration_sec` (defaults to 1.0 s if ≤ 0).

Returns `{status: "done", mode, duration_sec, linear_x, angular_z}`
serialized as JSON in the `status` field.

> ⚠️ **Precondition:** `move()` requires the robot to be in standing
> posture. If the pilot's tracked state is sitting (or unknown after
> a session restart), call `set_posture("stand")` before `move()`.

**Configurable via environment variables:**

| env var | default | meaning |
|---------|---------|---------|
| `LITE3_QUADRUPED_SPEED_MPS` | `0.2` | forward/backward speed (m/s) |
| `LITE3_QUADRUPED_ANG_SPEED_RPS` | `0.6` | rotation speed (rad/s) |
| `LITE3_QUADRUPED_CMD_DURATION_SEC` | `1.0` | default duration for raw velocity mode |
| `LITE3_ROTATE_TOLERANCE_DEG` | `3.0` | accepted closed-loop yaw error (degrees) |

## Required initialization

`rbnx boot` automatically runs the init → zero sequence during
primitive startup (in `on_init`).  No manual posture commands are
needed before moving — just call `set_posture("stand")` and the
robot is ready.

```
stand → (ready for move)
```

## Posture state machine (pilot must track)

```
              stand()/sit()
                   │
                   ▼
       ┌─────[STANDING]◄──────────┐
       │       │                   │
       │    move()                 │
       │       │                   │
       │       ▼                   │
       │   (moving)                │
       │       │                   │
       │  (move completes)         │
       │       │                   │
       └───────┴───────────────────┘
```

- `stand()` and `sit()` are the **same command** — they both put the robot
  into the standing/sitting posture. Do NOT call it again if already in
  that state.
- After a `move()` completes, the robot is still standing — do NOT
  re-issue `stand()` before the next `move()`.
- If the user says "sit down", call `set_posture("sit")` and update the
  tracked state. If the user then says "stand up", call
  `set_posture("stand")` (it's the same underlying command, but the pilot
  should issue it to follow the user's intent).

## Recommended pilot workflow

1. On first use after boot: `stand` (init + zero already done by `on_init`).
2. When the user says "go forward 1 meter": call `move(forward_m=1.0)`.
3. When the user says "turn left 90 degrees": call `move(rotate_deg=90)`.
4. When the user says "sit": call `set_posture("sit")` and mark state as
   sitting.
5. When the user says "stand" (and robot is sitting): call
   `set_posture("stand")` and mark state as standing.
6. When the user says "stand" (and robot is already standing): do nothing.

## Safety and preconditions

- **Before every `move()`, the robot MUST be standing.** If the current
  posture state is sitting (or unknown), call `set_posture("stand")` first.
  Do not call `move()` while the robot is sitting.
- Never issue `move()` before `stand` is complete.
- The robot sends a UDP heartbeat every 200 ms (`0x21040001`) in the
  background — this is managed automatically by the primitive after init.
- The transfer ROS2 nodes (`jetson2motion`, `jetson2app`, `sensor_checker`)
  must be healthy. If `/leg_odom2` is not available, the primitive enters
  `Deferred` state.
- `cmd_vel` publisher is created after init. If `move()` is called before
  init completes, it returns `{error: "ROS2 not initialized"}`.
- Default speeds are conservative (0.2 m/s linear, 0.6 rad/s angular).
  Consider the environment before moving.

## What this service does NOT do

- It does not provide continuous velocity control — each `move()` is a
  burst.
- It does not do obstacle avoidance or path planning. Use
  `nav2_wrapper_rbnx` for goal-based navigation.
- It does not provide odometry fusion — `/leg_odom2` is produced by the
  transfer nodes.
- It does not provide IMU or joint state directly — those are published
  on ROS2 topics by the transfer nodes.

## Lifecycle

The primitive is managed by `rbnx boot`. `on_init` spawns the transfer
ROS2 nodes, creates the `cmd_vel` publisher, waits for `/leg_odom2`,
starts the UDP heartbeat, then automatically sends `init` → `zero` UDP
commands to unlock and calibrate the robot. `on_shutdown` stops the
heartbeat and kills the transfer subprocess.

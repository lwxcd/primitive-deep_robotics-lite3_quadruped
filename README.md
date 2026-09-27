# lite3_quadruped

Lite3 chassis primitive with one capability surface and two selectable
backends: the physical motion host and the ROS 2/MuJoCo simulator.

## Backend selection

Set `backend` in the Robonix primitive config:

```yaml
# Physical robot
config:
  backend: hardware
  target_ip: 192.168.1.120
  target_port: 43893
  local_port: 43897
  app_port: 43899
  odom_topic: /odom

# MuJoCo bridge
config:
  backend: simulation
  odom_topic: /odom
  twist_in_topic: /cmd_vel
  sentinel_timeout_s: 30.0
```

`hardware` remains the compatibility default. The selected backend is logged
at startup. Simulation mode rejects `target_ip`, `target_port`, `local_port`,
and `app_port`; it never starts the hardware transfer process, heartbeat, or
Lite3 initialization sequence.

The simulation backend requires actively publishing `/clock`, `/odom`, and
`/joint_states`. Posture calls map to `/lite3_sim/stand`,
`/lite3_sim/lie_down`, `/lite3_sim/reset`, and
`/lite3_sim/emergency_stop` services.

Both backends expose the existing capabilities unchanged:

- `robonix/primitive/chassis/driver`
- `robonix/primitive/chassis/move`
- `robonix/primitive/chassis/twist_in`
- `robonix/primitive/chassis/odom`
- `robonix/primitive/chassis/posture`

Run backend unit tests with:

```bash
python3 -m unittest discover -s tests -v
```

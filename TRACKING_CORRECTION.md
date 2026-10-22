# Continuous bounded tracking correction — 2026-09-10

Implemented on franka-control in `~/franka_ManipForce-ROSAIC_ws/src/serl_franka_controllers`.
Record/replay Python and recorded trajectories were not changed.

The existing end-effector Jacobian fix is preserved. A real-time bounded force integrator now supplements the original position controller. It uses actual control-period seconds, keeps state across pose messages, and transfers pose commands through a realtime buffer. Invalid position/quaternion commands are rejected. Legacy translational_Ki and rotational_Ki are explicitly kept at zero because their prior sample-count integration/reset behavior is replaced.

Integration runs after 150ms with filtered target speed <=3mm/s and actual Cartesian speed <=15mm/s, for position errors <=15mm. Moving targets or larger errors smoothly release the added force. Only translation is integrated; rotation retains its existing controller.

Limits: gain 600 N/(m*s), additional force vector norm <=15N, vector slew <=3N/s. Estimated external force >=12N latches a trip and ramps the extra force down. This is an ADDITIONAL-force guard, not a total contact-force safety controller; the original impedance controller continues operating. A trip requires an explicit re-enable or controller restart. Disabling ramps down, rather than stepping force to zero.

## Controls

Source the workspace before using ROS commands:

```bash
source ~/franka_ManipForce-ROSAIC_ws/devel/setup.bash
rosservice call /cartesian_impedance_controller/tracking_correction/set_enabled 'data: false'
rosservice call /cartesian_impedance_controller/tracking_correction/set_enabled 'data: true'
```

`config/tracking_correction.yaml` is loaded by `impedance_gumi.launch`, with enabled=true on startup. Runtime service changes do not edit this YAML.

Diagnostic topic: `/cartesian_impedance_controller/tracking_correction/state` (`Float64MultiArray`, 20Hz).
Indices 0..2: added force XYZ (N); 3..5: filtered desired minus actual XYZ (m); 6..8: raw desired XYZ (m); 9: estimated external-force norm (N); 10: state (0 off, 1 integrating, 2 gated, 3 tripped); 11: requested enabled (0/1).

## Verification and limitations

- Pure C++ tests passed for rate/magnitude caps, time-step independence, trip latching, ramp-down, invalid input, moving-target gating, saturation reversal and a simple constant-disturbance model.
- Controller compiled and linked; ldd -r found no missing libraries or unresolved symbols.
- Actual robot: existing hold error fell from 0.651mm to mean 0.189mm during a 15-second enable test, with maximum added force 2.862N and no trip. No new pose or gripper commands were issued for this test.
- This does not establish removal of the recorded 1cm insertion error or successful insertion. The original replay timeline is unchanged. A short recorded pause can end before this deliberately slow correction converges; reaching a target is still not an implicit guarantee from an HTTP acknowledgement.

## Recovery evidence

Pre-change controller (already containing the EE Jacobian fix), header and binary are backed up under `~/franka_diagnostics/continuous_20260910_1750/original`. This directory also holds build logs, the deployment record and smoke-test samples. Restoring a binary requires a controlled controller-process restart: plugin reload alone left the previous shared library mapped in this environment. Do not restart the custom HTTP server merely to reload the controller; its startup homes the gripper.

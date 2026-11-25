# S2F-X200: experimental Gazebo dynamics model

Five motors and seven servos, with numerical properties sampled from the
S2F-X200 MATLAB aircraft model. Both modified PX4 versions use the same assets.

## Launch

```bash
cd ~/PX4/v1.16.1-mod  # or ~/PX4/v1.17.0-mod
make px4_sitl_simulinkctrl gz_s2f_x200
# With downward lidar:
make px4_sitl_simulinkctrl gz_s2f_x200_lidar_down
```

The Gazebo-owned presets are 22001 and 22002. Both source the same PX4 hardware
preset and use identical actuator routing. `HEADLESS=1` is also supported by the
normal PX4 launcher. Select the desired preset at startup; persisted parameter
values can override preset defaults.

## Mass, CG and propulsion

- Total mass: **7.6 kg**, including the moving rotor and surface links.
- CG in body FRD: **[-0.0127000254, 0, +0.0381000762] m**. Gazebo uses FLU.
- Full inertia about CG, in FLU, in kg m²:
  `[[0.888,-0.002,-0.071],[-0.002,1.107,0],[-0.071,0,1.720]]`.
  The generator subtracts child-link inertia and parallel-axis contributions
  from the base so these are aggregate properties at neutral joint angles.
- Five hub positions, the four lift motors' **15° outward cant**, rotation
  signs, and 16-inch diameter come from the reference vehicle.
- Lift thrust and torque fit the active `DynamicInflowModel`/BEMT calculation,
  including `CALC_FM`'s download correction. Cruise fits
  `MP_Propeller_Model`. The quadratic approximation covers 3000–8000 RPM;
  reference samples and fit errors are stored in `matlab_data.json`.
- Lift reaction torque is about **0.02022 N m per N** of thrust; cruise is
  **0.02251 N m per N**. Rotation signs follow the reference force calculation,
  including the `-SpinDir` passed to the lift model. The earlier integration
  model reversed these signs. Cant-induced moments remain physically active;
  yaw authority is not determined by reaction torque alone.
- Lift motor rise/fall constants are approximately **88.6/83.4 ms**; cruise
  **95.1/89.7 ms**, replacing the original 25 ms values. They match the 63.2%
  response of a reference ESC PI / propeller-inertia calculation for ±250 RPM
  steps from 5000 RPM, using sampled steady propeller load torque and the
  reference 1.271 N m torque limit. This is a local first-order approximation;
  large steps, saturation and dynamic inflow are not reproduced exactly.
- Rotor polar inertia is 0.000371 kg m² (lift), 0.000386 kg m² (cruise).
  Gazebo visual rotation is slowed by ten; rotor-link inertia is multiplied
  accordingly to preserve angular momentum and spin-up reaction. Its neutral
  contribution is removed from the base tensor to retain the aggregate target.

The lidar variant adds the existing LW20 housing and a sensor link: **21 g**
total payload, giving **7.621 kg**. The housing is ahead of the fuselage;
its ray starts at FLU `[0.35, 0, -0.10]` m, clear of the housing and skids.
The lidar preset sets `EKF2_RNG_POS_X/Y/Z` to `[0.35, 0, 0.10]` m in body
FRD, matching the physical sensor offset from the base-link IMU.
It reports 50 Hz downward range over 0.1–100 m using the standard PX4 bridge's
`lidar_sensor_link/sensor/lidar/scan` subscription. No client changes are needed.

## Aerodynamic mapping

Gazebo's standard distributed LiftDrag systems represent seven wing/tail panels.
Panel areas and quarter-chord coordinates include the active strip model's
geometry corrections. Wing incidence is 6°, tail incidence is 0°. Section slopes
and offsets come from the current E1212 / N0012 tables, with finite-wing slope
corrections and the source wing lift scale of 0.75.

Pre-stall drag and pitch coefficients are effective distributed fits to the
active `Strip_Aero_Model` **including NLS**, sampled at 20 m/s and -3°, 0°, +3°
body angle of attack. The committed checks compare lift/drag/pitch against those
source samples. This also supplies local rotational-flow effects at each panel.
The standard plugin lacks an independent zero-lift drag term, so its drag curve
cannot reproduce the full MATLAB polar away from the fitted conditions.

Control lift/sideforce and horizontal-surface pitch effectiveness are sampled
from the active model at ±2° deflection and 20 m/s. Positive wing/elevator
commands now produce the reference upward force; the old model used the
opposite sign. Channels remain: left aileron, left flap, right flap, right
aileron, left elevator, right elevator, rudder. Each joint spans ±30°.

This is a closer simulation approximation, not a port of the full nonlinear
model. Propwash, full Reynolds/Mach dependencies, unsteady inflow, nonlinear
control interactions, exact lateral control moments and dynamic derivatives,
post-stall flight, and the MATLAB rudder-to-boom-tilt coupling remain
approximations or are unimplemented. Visual geometry is schematic. Closed-loop
hover, yaw response and both VTOL transitions still require flight testing with
your controller before using this model for gain or performance validation.

## Actuator convention

PX4 motor channels 0–4 are four lift rotors followed by cruise; functions
101–105 map to `SIM_GZ_EC` outputs 1–5. The speed ceiling remains 838 rad/s
(about 8000 RPM), an integration assumption rather than a measured hardware ESC
limit. With the default zero thrust-curve factor, normalized commands are speed
fractions and thrust is quadratic. Controllers publishing normalized thrust
must convert to speed or configure a consistent motor curve.

Servo functions 201–207 map to `servo_0..6`. Neither actuator normalization nor
controller code is changed here. The publisher must still supply the expected
`vehicle_thrust_setpoint` and `vtol_vehicle_status` information. See PX4's
`src/modules/simulink_parameters/CONTROLLER_INTEGRATION.md` for that contract.

## Regeneration and checks

Gazebo needs no MATLAB, SciPy, external meshes, or source-machine paths to run
these committed assets. Regenerating SDF and checking numerical properties uses
Python's standard library:

```bash
python3 models/s2f_x200/generate_model.py
python3 models/s2f_x200/validate_model.py
SDF_PATH="$PWD/models" gz sdf -k models/s2f_x200_lidar_down/model.sdf
gz sdf --inertial-stats models/s2f_x200/model.sdf
```

To refresh the numeric snapshot from a newer reference, the optional offline
export uses MATLAB, NumPy and SciPy:

```bash
python3 models/s2f_x200/update_reference.py '/path/to/S2F-X200' --matlab matlab
```

`export_reference.m` samples read-only inputs in a temporary directory.
`update_reference.py` records source-file hashes, computes fits and regenerates
SDF. It never saves or alters the reference vehicle. Refreshes require reviewing
fits and signs again if the source model's conventions or topology change.

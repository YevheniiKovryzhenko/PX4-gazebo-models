# S2F-X200: experimental Gazebo integration model

This is a representative five-motor/seven-servo model for testing PX4/Simulink
interfaces, state reporting, and controller logic before hardware deployment.
It is not a validated replacement for the MATLAB nonlinear aircraft model.

## Contents and launch

This Gazebo repository owns both `models/s2f_x200/` and
`airframes/22001_gz_s2f_x200`. The matching modified PX4 repository discovers
fork-owned airframes, packages them into SITL, and creates the launch target:

```bash
cd ~/PX4/v1.16.1-mod
make px4_sitl_simulinkctrl gz_s2f_x200
# Or:
HEADLESS=1 make px4_sitl_simulinkctrl gz_s2f_x200
```

The shared PX4 hardware airframe supplies the vehicle type and logical channel
counts. The fork-owned Gazebo airframe selects this model and assigns only the
simulator output functions. Physical hardware routing is unaffected.

`matlab_data.json` contains the numeric source snapshot and source SHA-256.
`generate_model.py` regenerates `model.sdf` using Python's standard library and
the existing `models/standard_vtol/model.sdf` sensor definitions:

```bash
python3 models/s2f_x200/generate_model.py
gz sdf -k models/s2f_x200/model.sdf
```

No MATLAB installation, external meshes, or absolute source-machine paths are
needed to run the committed model. Visuals and collisions use basic shapes.

## Source-derived properties

The source is `Vehicle.mat` from the S2F-X200 MATLAB simulation folder, with
propeller polynomial interpretation checked against `MP_Propeller_Model.m`.

- Total articulated model mass: 7.6 kg.
- Body inertia diagonal: 0.888, 1.107, 1.720 kg m², with the stored products
  of inertia transformed from body FRD to Gazebo FLU.
- CG, five rotor positions, lift-rotor cant angles, rotation signs, and 16-inch
  propeller diameter come from the stored MATLAB vehicle.
- Span: 2.03 m; reference wing area: 0.50344 m².
- Seven surface channels preserve the model order: left aileron, left flap,
  right flap, right aileron, left elevator, right elevator, rudder.
- Panel areas and quarter-chord positions use the stored strip geometry.
- Small-angle lift slopes and zero-lift offsets are fit from the stored E1212
  and N0012 section tables over -3 to +3 degrees, with an approximate finite-wing
  correction. Stall angles use the section table's peak lift location.
- Static propeller coefficients approximate the MATLAB thrust/torque polynomials
  over 3000–8000 RPM with Gazebo's quadratic speed model. Lift motors use about
  `6.892e-5 N/(rad/s)^2`; cruise uses about `8.226e-5 N/(rad/s)^2`.

## Actuator interface

PX4 `actuator_motors.control[0..4]` maps to Gazebo motor channels 0–4: four lift
rotors followed by the cruise propeller. The simulator airframe maps functions
101–105 to `SIM_GZ_EC` outputs 1–5 and sets an 838 rad/s ceiling (approximately
8000 RPM). This ceiling is an integration-model assumption, not a hardware ESC
limit. With PX4's default motor thrust-curve factor of zero, a normalized command
is approximately a speed fraction; thrust is consequently quadratic in that
command. If the controller publishes normalized thrust instead, configure a
consistent motor curve or convert thrust to speed fraction in the controller.
Do not silently mix these conventions.

`actuator_servos.control[0..6]` maps to `servo_0..6` through functions 201–207
and `SIM_GZ_SV` outputs 1–7. Each simulated joint spans -30 to +30 degrees;
normalized zero is center. Servo 7 is the rudder slot; send zero or leave the
channel unused if the controller does not use it. Check each surface's physical
sign against the model's servo-mix parameters before closed-loop flight.

The airframe sets `MPC_USE_HTE=0` and nominal `MPC_THR_HOVER=0.4`. Its publisher
must still report normalized body thrust through `vehicle_thrust_setpoint` and
actual VTOL state through `vtol_vehicle_status`. The Gazebo model does not
synthesize controller-status messages or a controller watchdog.

The usual stock Gazebo VTOL ESC-current workarounds (`FD_ACT_EN=0`,
`FD_ESCS_EN=0`) are scoped to this simulation airframe. They do not change the
hardware airframe's failure-detector defaults.

## Approximations and validation scope

Wing/flap segmentation, simple visual geometry, servo dynamics, lift-to-control
angle effectiveness, post-stall behavior, body drag, propwash, dynamic derivatives,
and airspeed-dependent propeller performance are approximations. The section
fits do not reproduce the complete nonlinear MATLAB aerodynamic database.
Fin geometry is represented as a vertical surface; panel forces are applied to
the body at their representative centers. Small articulated-link inertias add
a small contribution on top of the specified body inertia. These choices are
intended for interface and logic testing, not controller-gain validation or
performance prediction.

SDF validation and a headless Gazebo load/step test check model structure and
plugin loading. A closed-loop flight with the real Simulink controller, including
sign checks, stationary hover, both VTOL transitions, landing, and failure cases,
is still required. The PX4 repository's
`src/modules/simulink_parameters/CONTROLLER_INTEGRATION.md` documents the message
contract and test cases.

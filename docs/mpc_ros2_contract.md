# ROS 2 MPC integration contract

The ROS 2 controller accepts the embedded `wolf_planner` through its existing
`EXT` mode. Build the matching `wolf_wbid` and `wolf_controller_core` revisions:
the planned-contact API and named postural reference receiver are coordinated
changes. Starting the controller alone keeps the WPG workflow.

## Observation

`wolf_controller/mpc_observation` is a relative topic in the robot namespace,
with reliable depth-one QoS and `wolf_msgs/msg/MpcObservation` messages. The
controller exposes `mpc_contract_version: 2` and `mpc_world_frame: world` for
planner compatibility checks. Both describe the actual transport and reject
configuration overrides or runtime changes.

| Field | Layout |
| --- | --- |
| `state.value[0:6]` | Linear and angular centroidal momentum divided by robot mass, in world axes |
| `state.value[6:9]` | Floating-base world position |
| `state.value[9:12]` | Unwrapped yaw, pitch, roll (ZYX Euler angles) |
| `state.value[12:24]` | Twelve leg joint positions in `mpc_joint_names` order |
| `input.value[0:12]` | Four world contact forces (xyz), in `mpc_contact_names` order |
| `input.value[12:24]` | Twelve joint velocities in the same joint order |
| `mode` | OCS2 contact bits: LF=8, RF=4, LH=2, RH=1 |
| `time` | Accumulated controller update periods in seconds, including dropped publications |

Slice endpoints in this table are exclusive. This corrects the previous
48-input publication and the incorrect LH/RF contact bits. The first six state
entries are normalized momentum, not a replacement vector of base velocities.

`mpc_contact_names` defaults to feet with the `lf`, `lh`, `rf`, `rh` prefixes,
in that order. `mpc_joint_names` defaults to the SRDF leg joints in the same limb
order, using each joint's actual model index. Arm joints are excluded. Set both
parameters explicitly when your naming or planner model order differs. They
must match the planner's task configuration exactly; the observation message
does not carry joint names. These configuration parameters cannot be changed
while the controller is configured. An invalid mapping disables only MPC
observation publication and emits a warning; the normal controller remains
available.

`wolf_controller/contact_forces` now has the `world` frame for its positions,
measured forces and desired forces. Consumers that previously assumed base-frame
positions must use the message frame or transform explicitly. In EXT, desired
contact flags describe the inverse-dynamics contact constraints.

## References and lifecycle

References remain under `wolf_controller/reference/` in the robot namespace:
`<foot>`, `<foot>_wrench`, `waist`, and `postural`. Cartesian and force references
use `world`. Postural messages contain explicit joint names and finite position
and velocity arrays; omitted arm joints keep their existing references.

The additional `contacts` topic uses `wolf_msgs/msg/ContactForces`. Fill `name`
with each of the robot's four foot names exactly once, and fill `des_contact`
with the corresponding planned stance flags. This is the contact mode at the
evaluated policy time, rather than the measured contact mode. Partial, duplicate,
and unknown names are rejected atomically. Other message fields are not used.
Publish the complete reference set, including contacts, before requesting MPC.

Use the existing `wolf_controller/set_control_mode` service
(`wolf_msgs/srv/String`, `data: MPC` to enter, `data: WPG` to leave). `MPC` is a
guarded service action that selects the existing EXT mode; state telemetry still
reports `EXT`. It requires ACTIVE state, a valid observation mapping, a fresh
contact command already adopted by the controller thread and fresh, validated
references on all ten WBID channels: four
foot Cartesian tasks, four point-force tasks, waist and postural. Postural
readiness requires every leg joint, while arm joints may be omitted. No CoM
reference is required by the EXT task stack. Missing first deliveries, stale
references and an invalid mode return `success: false`.

Each accepted reference must have arrived within
`mpc_contact_reference_timeout` seconds (default 0.25). Age uses steady receipt
time, so ROS clock changes cannot extend the deadline. Cartesian and wrench
messages need explicit `world` frames to count toward MPC readiness. Cartesian
pose and twist share a buffer, and reset generations prevent an in-flight old
callback from restoring readiness. A prepared MPC stream survives the WPG-to-EXT
task reset. Contact adoption also prevents interactive marker callbacks from
replacing MPC Cartesian references.

The separate DDS topics do not form an atomic policy snapshot. These guards
bound the age of each channel and validate its dimensions and values; they do
not prove that all channels came from the same solve or publisher. Planner
publication stamps can support diagnostics, but are not an atomicity guarantee.

While active EXT uses an adopted contact stream, expiry of contacts **or any
individual reference channel** calls the existing
`ControllerCore::emergencyStop()` on the controller thread and requests WPG.
The existing ANOMALY state zeros desired joint velocities and executes stand-down
impedance before returning to IDLE. This is the controller's existing emergency
response, not a hardware power-off. Fresh messages alone do not restart the
robot. A mere WPG request is insufficient because normal mode changes wait for
all feet to be in contact. If a requested EXT transition is still pending in WPG
when its prepared references expire, it is canceled before the core update.

Legacy `data: EXT` clients that have never provided the new contact stream
retain their previous entry behavior. Once a contact stream has been adopted,
EXT and the mode-toggle service use the same readiness checks. Starting the
controller alone never selects MPC or EXT automatically.

## Validation

`test/mpc_contract_test.cpp` runs without ROS and checks the 24/24 layout,
individual contact bits, both trot diagonals, reordered names, invalid command
rejection, entry prerequisites, active timeout stops and cancellation of a
deferred EXT request. Build it directly with C++17, or enable
`BUILD_TESTING` in the package build.

The coordinated migration still requires a ROS 2 Humble/Jazzy package build and
simulation checks for standing, both diagonal phases, controller pause/restart,
invalid observations, planner loss and the actual emergency transition. Do not
infer robot readiness from the standalone contract tests.

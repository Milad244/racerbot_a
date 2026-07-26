# Racerbot A Driving-Code Audit

**Audit date:** 2026-07-25  
**Audited commit:** `66020354af7f6c7924a647025fd2eab47456796c`  
**Scope:** `src/reactive`, `src/ftg_node`, and `src/disparity_extender`, compared with the ROS 2 Jazzy workspace's checked-in hardware, topic, mux, and safety configuration.

## Executive verdict

**Not ready for physical driving.** The packages build on ROS 2 Jazzy and the
default `reactive_launch.py` route does place `safety_node` between the
controller and `/drive`. However, there are release-blocking safety and vehicle
integration problems:

1. the controller can publish motion directly to `/drive` without a live LB
   deadman signal when it is run outside the composition launch;
2. the TTC safety node listens to the simulator topic
   `/ego_racecar/odom`, while this car publishes `/odom`, so TTC is inactive on
   the physical vehicle;
3. the default straight-line command is about **4.326 m/s**, with no configured
   maximum-speed clamp; and
4. the steering range substantially exceeds this car's calibrated actuator
   range.

Do not floor-test this implementation in its current state. The findings below
should be resolved and covered by automated tests before following the
workspace's wheels-off-ground and low-speed physical test ladder.

## Severity definitions

- **Critical:** can permit unintended motion or defeat a required safety layer;
  blocks physical testing.
- **High:** can produce unsafe or materially incorrect driving; should block
  physical testing until resolved.
- **Medium:** reliability, maintainability, or integration weakness that should
  be addressed before wider use.
- **Low:** cleanup or documentation issue with limited immediate runtime impact.

## Findings

### A-01 — Critical — The drive-producing node has no mandatory deadman gate

Evidence:

- `src/reactive/src/gap_follow_node.cpp:7` creates a `drive` publisher.
- `src/reactive/src/gap_follow_node.cpp:94-98` publishes non-zero Ackermann
  commands without subscribing to `/joy` or checking LB.
- `src/reactive/launch/reactive_launch.py:53-70` remaps this publisher to
  `drive_raw`, and `safety_node` normally gates that stream. The protection
  therefore depends on using this exact launch composition.
- `ros2 run reactive gap_follow_node` bypasses the wrapper and publishes
  directly to `/drive`. An isolated synthetic `Gap` test, with no `/joy`
  publisher at all, produced:

  ```text
  steering_angle: ~0.0 rad
  speed: 4.326356 m/s
  ```

Impact: running, reusing, remapping, or composing the executable directly can
move the physical car without LB. This violates the workspace policy that every
node capable of moving the car independently enforce a fresh LB hold.

Required direction: put the live `/joy`/LB check in every executable that can
publish a motion command. A safety-wrapper composition can remain as
defence-in-depth, but it must not be the only deadman gate.

### A-02 — Critical — TTC uses a simulator odometry topic, not this car's topic

Evidence:

- `src/reactive/src/safety_node.cpp:25-26` subscribes to
  `/ego_racecar/odom`.
- The physical workspace publishes `nav_msgs/Odometry` on `/odom` from
  `vesc_to_odom_node`; no default component publishes `/ego_racecar/odom`.
- `speed_` therefore remains at its initialized `0.0f`
  (`src/reactive/include/safety_node.hpp:51`).
- `scan_callback()` skips TTC whenever closing speed is effectively zero
  (`src/reactive/src/safety_node.cpp:77-80`).
- In an isolated runtime check, publishing 3.0 m/s on the car's `/odom` found
  no subscriber. After a 0.1 m forward obstacle scan and a held synthetic LB
  signal, `safety_node` forwarded a 2.0 m/s `/drive_raw` command unchanged.
  The Joy timeout was lengthened only to isolate TTC behavior.

Impact: the node named and documented as the safety node provides no
time-to-collision braking on the physical car.

Required direction: make the odometry topic a declared parameter defaulting to
`/odom`, validate odometry freshness, and add an integration test using the
physical topic graph.

### A-03 — Critical — Default speed reaches 4.326 m/s with no speed clamp

Evidence:

- `src/reactive/src/gap_follow_node.cpp:147-150` uses
  `-1.6 * log(angle + 0.3) + 2.4`.
- At zero steering this is 4.326356 m/s, confirmed by the isolated runtime
  test.
- The result is not clamped and there is no `max_speed` parameter.
- The checked-in VESC limit corresponds to about 5.039 m/s
  (`23250 ERPM / 4614 ERPM-per-m/s`), so the downstream driver does not reduce
  a 4.326 m/s request.
- The fallback controller also uses fixed speeds as high as 3.0 m/s
  (`src/reactive/src/gap_follow_node.cpp:40-50`).

Impact: the first valid straight gap can request near-racing speed, including
on an unvalidated track and before the controller's geometry is proven on the
car.

Required direction: add conservative, validated min/max speed parameters and
clamp every output. Initial physical validation should use the workspace's
low-speed values, not the present formula maximum.

### A-04 — High — FOV masking creates division by zero in obstacle extension

Evidence:

- `preprocess_lidar()` sets beams outside ±90 degrees to exactly zero
  (`src/reactive/src/gap_finder_node.cpp:63-70`).
- At an FOV boundary, an open in-FOV beam commonly differs from that artificial
  zero by more than `disparity_threshold_`.
- `extend_obstacles()` then chooses `closer_range == 0` and divides by it at
  lines 91-94 before converting the non-finite result to `size_t`.
- A uniform open full-circle synthetic scan produced "No valid gap found,"
  demonstrating that artificial boundary values can eliminate otherwise open
  space. The exact amount of corruption is scan-resolution dependent.

Impact: a clear scene can yield no command, and the floating-to-integer
conversion has undefined/implementation-dependent behavior.

Required direction: crop the processing window without presenting mask zeros
as physical disparities, and reject non-positive/non-finite denominators before
angle-to-index conversion.

### A-05 — High — NaN LaserScan values are not sanitized and can stall gap search

Evidence:

- `preprocess_lidar()` only applies `std::min`; it never checks
  `std::isfinite` or `range_min` (`src/reactive/src/gap_finder_node.cpp:59-73`).
- A NaN survives the `std::min` call.
- In `find_furthest_gap()`, both `NaN <= threshold` and `NaN > threshold` are
  false (`src/reactive/src/gap_finder_node.cpp:115-131`). The index is then not
  advanced, while width arithmetic also consumes the NaN.

Impact: a malformed or degraded scan can hang the callback, consume a CPU core,
and stop controller updates. The mux should eventually time out the old
command, but the controller itself is no longer healthy or diagnosable.

Required direction: sanitize every beam using `range_min`, `range_max`, and
finite checks; ensure every loop path advances; add NaN, infinity, zero, empty,
and malformed-scan tests.

### A-06 — High — Steering commands exceed this car's calibrated range

Evidence:

- `src/reactive/include/gap_follow_node.hpp:27` allows ±90 degrees
  (±1.571 rad).
- This car maps steering to servo position as
  `servo = -1.2135 * angle + 0.5304`, with servo limits 0.15 to 0.85.
- Those checked-in values reach the configured servo stops at approximately
  +0.313 rad and -0.263 rad.
- The VESC driver clips the resulting servo value, so much of the controller's
  requested range collapses to a saturated physical command.

Impact: the path model assumes steering changes that the actuator cannot
produce. Speed selection is then based on the requested angle rather than
known achievable steering, reducing predictability and creating discontinuous
control at the stops.

Required direction: use a car-specific steering limit at or inside the
calibrated range and validate the sign and asymmetric limits wheels-off-ground.

### A-07 — High — The fallback target is the first beam of the gap, not its best point

Evidence:

- Gap selection searches for the gap containing the furthest range
  (`src/reactive/src/gap_finder_node.cpp:144-153`).
- The published `target_angle` and `target_range` instead use `gap_start`
  (`src/reactive/src/gap_finder_node.cpp:53-54`).
- The fallback controller steers directly to that angle
  (`src/reactive/src/gap_follow_node.cpp:38-57`).
- The least-squares controller also invokes this fallback automatically when
  there are too few points (`src/reactive/src/gap_follow_node.cpp:63-67`).

Impact: fallback behavior aims at a gap edge—typically adjacent to the inflated
obstacle—rather than the gap center or furthest safe point.

Required direction: explicitly publish and test a center/best-point target that
maintains vehicle-footprint clearance.

### A-08 — High — Control outputs and parameters are not validated

Evidence:

- `degree`, `steering_gain`, and `lookahead_distance` accept arbitrary values
  without bounds (`src/reactive/src/gap_follow_node.cpp:13-23`).
- The code assumes `angles.size() == ranges.size()` but does not check it
  (`src/reactive/src/gap_follow_node.cpp:74-79`).
- Polynomial coefficients, target output, steering, and speed are not checked
  for finite values before publication.
- A negative degree, malformed `Gap`, ill-conditioned fit, or non-finite input
  can cause invalid Eigen dimensions, out-of-bounds access, or NaN actuator
  commands.

Impact: parameter mistakes or bad intermediate data can turn into crashes or
invalid commands instead of a fail-safe zero-speed command.

Required direction: validate parameters at startup, validate every received
message, and refuse to publish motion unless the final speed and steering are
finite and within explicit limits.

### A-09 — Medium — TTC behavior is too weakly specified for an emergency brake

Evidence:

- The default threshold is only 0.3 seconds
  (`src/reactive/src/safety_node.cpp:11`).
- The node publishes a zero Ackermann speed
  (`src/reactive/src/safety_node.cpp:83-88`); it does not command or verify a
  braking-current path.
- There is no odometry freshness check. A stale last speed can cause false
  stops; no odometry leaves TTC disabled as described in A-02.
- The estop state is reset at the start of every scan
  (`src/reactive/src/safety_node.cpp:64-68`) rather than being explicitly
  latched or released according to documented hysteresis.

Impact: even after the topic is corrected, stopping distance, state release,
and stale-data behavior are not established well enough to call this an
emergency brake.

Required direction: define and test the stopping contract at each permitted
speed, include sensor freshness and hysteresis, and verify actual stopping
distance on stands and then at low speed.

### A-10 — Medium — The repository has no functional regression tests

Evidence:

- No `test/` source files exist in this repository.
- The gap geometry, polynomial path, speed curve, deadman gate, TTC, topic
  wiring, malformed input, and actuator clamps have no automated behavioral
  coverage.
- The configured lint suite does not fully pass:
  - `reactive`: 3 of 6 lint tests failed (`flake8`, `lint_cmake`,
    `uncrustify`);
  - `ftg_node`: 2 of 6 failed (`flake8`, `uncrustify`);
  - `disparity_extender`: no tests were configured.
- The Jazzy build emitted signed/unsigned comparison warnings in
  `gap_finder_node.cpp:47` and `gap_follow_node.cpp:74`.

Impact: safety regressions and integration mistakes can merge without a
behavioral signal.

Required direction: extract ROS-free control math for unit tests and add ROS
integration tests for topic/QoS/deadman/TTC behavior before physical use.

### A-11 — Medium — Two installed packages are skeletons, not usable controllers

Evidence:

- `ftg_node` installs and launches an executable, but its LaserScan callback is
  empty (`src/ftg_node/src/ftg_node.cpp:14-16`). It advertises `drive` but never
  commands it.
- `disparity_extender` contains only build metadata and installs no executable.
- `ftg_node/package.xml` still contains TODO description/license fields.

Impact: operators can successfully build or launch packages that do no driving,
which complicates race-day diagnosis and makes package readiness unclear.

Required direction: clearly label skeleton packages as non-operational or
remove them from operator-facing workflows until implemented and tested.

## Positive observations

- All four audited packages (`reactive`, `ftg_node`,
  `disparity_extender`, and Racerbot B's separately located
  `gap_follow_node`) compiled together on the installed ROS 2 Jazzy toolchain.
- `rosdep check` reports all Racerbot A system dependencies satisfied.
- The default `reactive_launch.py` composition remaps controller output to
  `/drive_raw`, enables the safety node's deadman gate by default, checks LB at
  button index 4, and expires Joy input after 0.5 seconds.
- The final command path uses `AckermannDriveStamped` and `/drive`, so it enters
  `ackermann_mux` rather than bypassing the mux or publishing low-level VESC
  commands.
- Published commands are timestamped.
- When the gap finder emits no new message, the callback-driven control stream
  goes quiet and the workspace mux's 0.2-second navigation timeout provides a
  downstream stale-command stop.

These strengths are useful foundations, but they do not offset the critical
findings above.

## Verification performed

All runtime checks used isolated ROS domain IDs and only synthetic messages.
The hardware bringup, VESC, and motors were not started.

| Check | Result |
|---|---|
| ROS 2 Jazzy isolated build | Pass; all audited packages built |
| `rosdep check --from-paths src --ignore-src` | Pass |
| Configured lint tests | Fail; details in A-10 |
| Functional/unit tests | None present |
| Direct controller with no `/joy` | Published 4.326356 m/s |
| Safety node subscriber on physical `/odom` | No subscriber |
| TTC scenario using physical `/odom` | 2.0 m/s raw command forwarded |
| Uniform open full-circle scan to gap finder | No valid gap produced |
| Physical-car test | Not performed; unsafe at this audit state |

## Minimum exit criteria before physical testing

1. Every motion-producing executable independently enforces a fresh LB hold.
2. TTC consumes `/odom`, validates freshness, and has passing stop/release tests.
3. Speed and steering are explicitly clamped to conservative car-specific
   values.
4. LaserScan and `Gap` data are validated, including NaN/Inf/empty/malformed
   cases.
5. Open space selects a stable near-straight target; gap-edge fallback is
   replaced.
6. Unit and ROS integration tests cover the findings above and the configured
   test suite passes.
7. Only then: static topic inspection, wheels off the ground, and low-speed
   floor testing in an open area with an operator at the power disconnect.


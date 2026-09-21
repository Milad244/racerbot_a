# Research

Contains notes and learnings from testing in simulation, on the car, and from lectures, so we can track what's worked, what hasn't, and why

## Simulation

- If our turning in simulation is not consistent with our turning in real life testing, the likely culprit to consider would be the simulator's vehicle dynamics model.
    - If nonlinear: We may be cornering too fast, causing saturation/sliding that wouldn't be picked up with a kinematic or linear model.

## Safety
- TTC needs reliable odometry.
- Safety should be built into the driving controller, not a separate process, otherwise they compete with each other.

## Reactive Driving

### Follow the Gap (FTG)
- Preprocess lidar scan
    - Restrict scan to 180 degrees: Good since it removes lidar points behind and to the sides of the car that aren't relevant to forward driving.
    - Smooth by taking mean every 3 scans: Don't know if it makes a difference.
    - Remove outlier scans: To be tested.

- Safety bubble
    - Using exact value instead of linear approximation is good.
    - Overall, good to avoid crashing into corners.

- Finding Gap
    - We choose widest gap: Backfires when a wide but shallow gap wins over a narrower, deeper one, steering us toward a dead end.
    - We choose width*depth gap: Solid.
    - We account for how much we must turn: To be tested.
    - We choose center of widest gap: Solid.

- Steering
    - Go to best point: Car oversteers, as it chooses a new point each time.

- Speed
    - Slower speeds generally crash less and are closer to matching real life vs. simulation.
    - No solid speed function yet.

### Disparity Extender (DE)

- Decreasing the speed caused the car to crash into more corners. Added saftey bubble to try and counteract this. Good


### Least Squares

(No notes yet.)

## Mapping Driving

(No notes yet.)

## AI Driving

(No notes yet.)

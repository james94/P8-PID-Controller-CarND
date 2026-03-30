# Problem Statement: C++ Steering and Throttle PID Controller Backend for Udacity Unity Simulator

## Background
Autonomous driving systems must continuously convert noisy, real-time telemetry into stable control commands that keep a vehicle safe and on-path. In this project, a C++ backend application is integrated with the Udacity Unity 3D Lake Track simulator to control a single simulated vehicle using closed-loop feedback. The backend receives telemetry over a websocket interface, processes cross-track and speed-related error signals, and sends steering and throttle commands back to the simulator. The project is evaluated under a practical safety criterion used in the Self-Driving Car Nanodegree: complete at least one full lap without leaving the drivable roadway or exhibiting unsafe vehicle behavior.

## Core Problem
The core problem is to deliver reliable lateral and longitudinal control in a simulator-driven autonomy loop where both controls interact. The application must simultaneously manage steering corrections to reduce cross-track error and throttle adjustments to regulate speed, while avoiding instability, oscillation, and loss of control. Because steering and throttle directly influence one another during turns and recovery maneuvers, the backend must produce balanced control outputs that preserve track adherence throughout a continuous lap.

## Operational Context
The system operates as a C++ process that communicates with the Udacity Unity simulator through uWebSockets telemetry messages. Incoming data includes cross-track error (CTE), vehicle speed, and current steering angle. Two PID control paths are used at runtime: one dedicated to steering and one dedicated to throttle/speed management. The backend computes control actions from telemetry-derived error, applies actuator-range limits expected by the simulator, packages commands into JSON, and transmits those commands to drive the simulated car in real time.

## System Boundaries (In Scope / Out of Scope)
In scope:
- Real-time processing of simulator telemetry and generation of steering and throttle commands.
- Closed-loop PID-based control behavior for lateral and longitudinal vehicle regulation.
- Safety-focused validation against one-lap track-completion requirements.

Out of scope:
- Perception, localization, mapping, and route-planning subsystems.
- Hardware deployment, sensor fusion stacks, and real-vehicle actuation.
- Advanced adaptive or learning-based control methods beyond the implemented PID backend.

## Constraints and Assumptions
The solution assumes a stable websocket connection to the simulator and correctly formatted telemetry events. Command outputs must remain within simulator-compatible actuator limits to avoid invalid control requests. Evaluation is constrained to simulator dynamics, fixed track geometry, and the project rubric definition of safe completion. The backend is expected to run deterministically enough to support responsive control within the simulator update loop.

## Success Criteria
Success is defined by demonstrating that the C++ backend can command the simulated vehicle to complete at least one full lap of the Udacity lake track while keeping all tires on the drivable surface, avoiding ledge pop-ups, and preventing rollover or other unsafe behavior. The controller must maintain both steering and throttle authority throughout the run and sustain stable behavior rather than momentary or partial completion.

## Why This Problem Matters
This problem captures a foundational autonomy challenge: turning real-time error signals into dependable motion control under safety constraints. It validates essential control-engineering skills, backend integration discipline, and simulator-in-the-loop testing practices that are directly relevant to larger self-driving system development. A successful result demonstrates readiness to design and evaluate feedback control modules that can be integrated into broader autonomous driving pipelines.

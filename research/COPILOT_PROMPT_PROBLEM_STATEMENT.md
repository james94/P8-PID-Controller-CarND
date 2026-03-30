## Prompt For GitHub Copilot: Generate Project Problem Statement

You are a senior autonomous-systems technical writer. Create a clear, professional problem statement for my C++ backend project that controls steering and throttle in the Udacity Unity 3D Lake Track simulator.

Use the following project context as source truth:

- Project purpose: implement a PID-based controller that safely drives a simulated car around the lake racetrack.
- Runtime integration: C++ backend communicates with Udacity simulator using uWebSockets and telemetry events.
- Telemetry inputs consumed by backend: crosstrack error (CTE), speed, steering angle.
- Controller architecture:
	- One PID controller for steering.
	- One PID controller for throttle (speed control).
- Steering control behavior:
	- PID updates from CTE.
	- Steering command is clamped to [-1, 1].
	- Tuned coefficients used in code: Kp=0.10, Ki=0.0001, Kd=1.0.
- Throttle control behavior:
	- Uses speed error based on desired speed of 30 mph.
	- Throttle command is clamped to [0, 1].
	- Tuned coefficients used in code: Kp=0.1, Ki=0.00015, Kd=0.0.
- Safety/performance target: complete at least one full lap without leaving the drivable track surface, without popping onto ledges, and without rollover/unsafe behavior.

Output requirements:

1. Write in Markdown.
2. Title: "Problem Statement: C++ Steering and Throttle PID Controller Backend for Udacity Unity Simulator".
3. Include these sections in order:
	 - Background
	 - Core Problem
	 - Operational Context
	 - System Boundaries (In Scope / Out of Scope)
	 - Constraints and Assumptions
	 - Success Criteria
	 - Why This Problem Matters
4. Keep it concise but specific (roughly 300 to 500 words).
5. Keep language implementation-aware but not code-heavy.
6. Do not include solution design steps, pseudocode, or tuning instructions.

Quality bar:

- Should read like a submission-ready engineering writeup for a Self-Driving Car Nanodegree project.
- Must explicitly mention both steering and throttle control responsibilities.
- Must explicitly reference the simulator coupling and one-lap safety objective.
- Should be understandable by reviewers who did not read the source code.

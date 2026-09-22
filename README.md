# Nashoba Robotics 2026

### FRC Team 1768 — Nashoba Robotics

Competition robot software for the **2026 FIRST Robotics Competition season**.

The 2026 robot combines a swerve drivetrain, vision-assisted localization, automated shooting, and a coordinated game-piece handling system.

---

## Software Architecture

The project uses the WPILib command-based model with an IO-layer architecture that separates mechanism logic from real and simulated hardware.

    src/main/
    ├── java/frc/robot/
    │   ├── Main.java
    │   ├── Robot.java
    │   ├── RobotContainer.java
    │   ├── Constants.java
    │   ├── FieldConstants.java
    │   ├── Presets.java
    │   │
    │   ├── autos/
    │   │   └── Autonomous routines and constants
    │   │
    │   ├── commands/
    │   │   └── Drive and robot commands
    │   │
    │   ├── subsystems/
    │   │   ├── drive/
    │   │   ├── vision/
    │   │   ├── shooter/
    │   │   ├── hood/
    │   │   ├── intakedeploy/
    │   │   ├── intakeroller/
    │   │   ├── rollerfloor/
    │   │   ├── entryroller/
    │   │   ├── Superstructure.java
    │   │   └── LEDSubsystem.java
    │   │
    │   └── util/
    │       ├── ShootingUtil.java
    │       └── Utility and tuning classes
    │
    └── deploy/
        └── Runtime assets and trajectory files

### `Robot.java`

Controls the main robot lifecycle and AdvantageKit runtime configuration.

Depending on the selected mode, the robot can:

- Run on real hardware
- Run in desktop simulation
- Replay a previously recorded AdvantageKit log

Build information including the Git SHA, branch, build date, and dirty state is also written to the log.

### `RobotContainer.java`

The central composition root for the robot.

It is responsible for:

- Creating subsystems
- Selecting real, simulated, or replay IO implementations
- Configuring driver controls
- Configuring autonomous routines
- Exposing characterization utilities
- Wiring the drivetrain, vision, and superstructure together

### `Superstructure.java`

Coordinates mechanisms that need to operate together.

Rather than making the driver independently control the drivetrain, hood, shooter, and feeder during a shot, the superstructure combines those actions into higher-level commands such as:

    Aim
      ├── Rotate drivetrain toward target
      ├── Track calculated hood angle
      └── Track calculated shooter speed

    Shoot
      ├── Run roller floor
      └── Run entry roller

This keeps complex robot actions centralized and predictable.

---

## Acknowledgements

This project builds on software and ideas from the broader FRC community, including:

- WPILib
- AdvantageKit
- CTRE Phoenix
- PhotonVision
- PathPlanner
- Choreo

---

<div align="center">

### Nashoba Robotics · FRC Team 1768

</div>

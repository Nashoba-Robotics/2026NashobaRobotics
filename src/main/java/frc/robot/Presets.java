package frc.robot;

import frc.robot.util.LoggedTunableNumber;

public final class Presets {

  public static class Hood {
    public static final LoggedTunableNumber TUCK_ANGLE_DEG =
        new LoggedTunableNumber("Hood/Setpoints/TuckAngleDeg", 1.5);
    public static final LoggedTunableNumber CLOSE_HUB_ANGLE_DEG =
        new LoggedTunableNumber("Hood/Setpoints/CloseHubAngleDeg", 0.5);
    public static final LoggedTunableNumber TUNING_ANGLE_DEG =
        new LoggedTunableNumber("Hood/Setpoints/TuningAngleDeg", 0.0);
  }

  public static class RollerFloor {
    public static final LoggedTunableNumber FEED_SPEED =
        new LoggedTunableNumber("RollerFloor/Setpoints/FeedSpeed", 280.0);
    public static final LoggedTunableNumber SLOW_FEED_SPEED =
        new LoggedTunableNumber("RollerFloor/Setpoints/SlowFeedSpeed", 50.0);
    public static final LoggedTunableNumber EXHAUST_SPEED =
        new LoggedTunableNumber("RollerFloor/Setpoints/ExhaustSpeed", -75.0);
    public static final LoggedTunableNumber TUNING_SPEED =
        new LoggedTunableNumber("RollerFloor/Setpoints/TuningSpeed", 0.0);
  }

  public static class Intake {
    public static final LoggedTunableNumber TUCK_ANGLE_DEG =
        new LoggedTunableNumber("IntakeDeploy/Setpoints/DeployTuckAngleDeg", 0.0);
    public static final LoggedTunableNumber EXTEND_ANGLE_DEG =
        new LoggedTunableNumber("IntakeDeploy/Setpoints/DeployExtendAngleDeg", 0.0);
    public static final LoggedTunableNumber TUNING_ANGLE_DEG =
        new LoggedTunableNumber("IntakeDeploy/Setpoints/DeployTuningAngleDeg", 0.0);

    public static final LoggedTunableNumber INTAKE_VOLTS =
        new LoggedTunableNumber("IntakeRoller/Setpoints/RollerIntakeVolts", 8.0);
    public static final LoggedTunableNumber SLOW_INTAKE_VOLTS =
        new LoggedTunableNumber("IntakeRoller/Setpoints/RollerIntakeVolts", 3.5);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("IntakeRoller/Setpoints/RollerExhaustVolts", -6.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("IntakeRoller/Setpoints/RollerTuningVolts", 0.0);

    public static final LoggedTunableNumber INTAKE_SPEED =
        new LoggedTunableNumber("IntakeRoller/Setpoints/RollerIntakeSpeed", 240.0);
    public static final LoggedTunableNumber AUTO_INTAKE_SPEED =
        new LoggedTunableNumber("IntakeRoller/Setpoints/RollerIntakeSpeed", 260.0);
    public static final LoggedTunableNumber TUNING_SPEED =
        new LoggedTunableNumber("IntakeRoller/Setpoints/TuningSpeed", 0.0);
  }

  public static class EntryRoller {
    public static final LoggedTunableNumber FEED_SPEED =
        new LoggedTunableNumber("EntryRoller/Setpoints/FeedSpeed", 225.0);
    public static final LoggedTunableNumber EXHAUST_SPEED =
        new LoggedTunableNumber("EntryRoller/Setpoints/ExhaustSpeed", -70.0);
    public static final LoggedTunableNumber SLOW_EXHAUST_SPEED =
        new LoggedTunableNumber("EntryRoller/Setpoints/SlowExhaustSpeed", 0.0);
    public static final LoggedTunableNumber TUNING_SPEED =
        new LoggedTunableNumber("EntryRoller/Setpoints/TuningSpeed", 0.0);
  }

  public static class Shooter {
    public static final LoggedTunableNumber CLOSE_HUB_SPEED =
        new LoggedTunableNumber("Shooter/Setpoints/CloseHubSpeedRadsPerSec", 240.0);
    public static final LoggedTunableNumber EXHAUST_SPEED =
        new LoggedTunableNumber("Shooter/Setpoints/ExhaustSpeedRadsPerSec", -100.0);
    public static final LoggedTunableNumber TUNING_SPEED =
        new LoggedTunableNumber("Shooter/Setpoints/TuningSpeedRadsPerSec", 0.0);
  }
}

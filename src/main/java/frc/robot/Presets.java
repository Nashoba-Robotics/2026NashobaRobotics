package frc.robot;

import frc.robot.util.LoggedTunableNumber;

public final class Presets {

  public static class Climber {
    public static final LoggedTunableNumber TUCK_ANGLE_DEG =
        new LoggedTunableNumber("Climber/Setpoints/TuckAngleDeg", 0.0);
    public static final LoggedTunableNumber TUNING_ANGLE_DEG =
        new LoggedTunableNumber("Climber/Setpoints/TuningAngleDeg", 0.0);
  }

  public static class Hood {
    public static final LoggedTunableNumber TUCK_ANGLE_DEG =
        new LoggedTunableNumber("Hood/Setpoints/TuckAngleDeg", 1.5);
    public static final LoggedTunableNumber CLOSE_HUB_ANGLE_DEG =
        new LoggedTunableNumber("Hood/Setpoints/CloseHubAngleDeg", 10.0);
    public static final LoggedTunableNumber TUNING_ANGLE_DEG =
        new LoggedTunableNumber("Hood/Setpoints/TuningAngleDeg", 0.0);
  }

  public static class Spindexer {
    public static final LoggedTunableNumber FEED_VOLTS =
        new LoggedTunableNumber("Spindexer/Setpoints/FeedVolts", 12.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Spindexer/Setpoints/ExhaustVolts", 0.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("Spindexer/Setpoints/TuningVolts", 0.0);
  }

  public static class Intake {
    public static final LoggedTunableNumber TUCK_ANGLE_DEG =
        new LoggedTunableNumber("Intake/Setpoints/DeployTuckAngleDeg", 45.0);
    public static final LoggedTunableNumber EXTEND_ANGLE_DEG =
        new LoggedTunableNumber("Intake/Setpoints/DeployExtendAngleDeg", 120.0);
    public static final LoggedTunableNumber TUNING_ANGLE_DEG =
        new LoggedTunableNumber("Intake/Setpoints/DeployTuningAngleDeg", 0.0);

    public static final LoggedTunableNumber INTAKE_VOLTS =
        new LoggedTunableNumber("Intake/Setpoints/RollerIntakeVolts", 10.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Intake/Setpoints/RollerExhaustVolts", -6.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("Intake/Setpoints/RollerTuningVolts", 0.0);
  }

  public static class Loader {
    public static final LoggedTunableNumber FEED_VOLTS =
        new LoggedTunableNumber("Loader/Setpoints/FeedVolts", 10.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Loader/Setpoints/ExhaustVolts", 0.0);
    public static final LoggedTunableNumber SLOW_EXHAUST_VOLTS =
        new LoggedTunableNumber("Loader/Setpoints/SlowExhaustVolts", -6.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("Loader/Setpoints/TuningVolts", 0.0);
  }

  public static class Shooter {
    public static final LoggedTunableNumber CLOSE_HUB_SPEED =
        new LoggedTunableNumber("Shooter/Setpoints/CloseHubSpeedRadsPerSec", 250.0);
    public static final LoggedTunableNumber TUNING_SPEED =
        new LoggedTunableNumber("Shooter/Setpoints/TuningSpeedRadsPerSec", 0.0);
  }
}

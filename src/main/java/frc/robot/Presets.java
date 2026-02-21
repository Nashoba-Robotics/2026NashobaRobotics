package frc.robot;

import frc.robot.util.LoggedTunableNumber;

public final class Presets {

  public static class Climber {
    public static final LoggedTunableNumber TUCK_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Climber/TuckAngleDeg", 0.0);
    public static final LoggedTunableNumber TUNING_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Climber/TuningAngleDeg", 0.0);
  }

  public static class Hood {
    public static final LoggedTunableNumber TUCK_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Hood/TuckAngleDeg", 1.5);
    public static final LoggedTunableNumber CLOSE_HUB_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Hood/CloseHubAngleDeg", 10.0);
    public static final LoggedTunableNumber TUNING_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Hood/TuningAngleDeg", 0.0);
  }

  public static class Spindexer {
    public static final LoggedTunableNumber FEED_VOLTS =
        new LoggedTunableNumber("Setpoints/Spindexer/FeedVolts", 6.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Spindexer/ExhaustVolts", 0.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("Setpoints/Spindexer/TuningVolts", 0.0);
  }

  public static class Intake {
    public static final LoggedTunableNumber TUCK_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Intake/DeployTuckAngleDeg", 0.0);
    public static final LoggedTunableNumber EXTEND_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Intake/DeployExtendAngleDeg", 95.0);
    public static final LoggedTunableNumber TUNING_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Intake/DeployTuningAngleDeg", 0.0);

    public static final LoggedTunableNumber INTAKE_VOLTS =
        new LoggedTunableNumber("Setpoints/Intake/RollerIntakeVolts", 10.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Intake/RollerExhaustVolts", -6.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("Setpoints/Intake/RollerTuningVolts", 0.0);
  }

  public static class Loader {
    public static final LoggedTunableNumber FEED_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/FeedVolts", 10.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/ExhaustVolts", 0.0);
    public static final LoggedTunableNumber SLOW_EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/SlowExhaustVolts", -6.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/TuningVolts", 0.0);
  }

  public static class Shooter {
    public static final LoggedTunableNumber CLOSE_HUB_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/CloseHubSpeedRadsPerSec", 275.0);
    public static final LoggedTunableNumber TUNING_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/TuningSpeedRadsPerSec", 0.0);
  }
}

package frc.robot;

import frc.robot.util.LoggedTunableNumber;

public final class Presets {

  public static class Climber {
    public static final LoggedTunableNumber TUCK_ANGLE =
        new LoggedTunableNumber("Setpoints/Climber/TuckAngle", 0.0);
  }

  public static class Hood {
    public static final LoggedTunableNumber TUCK_ANGLE =
        new LoggedTunableNumber("Setpoints/Hood/TuckAngle", 0.0);
    public static final LoggedTunableNumber CLOSE_HUB_ANGLE =
        new LoggedTunableNumber("Setpoints/Hood/CloseHubAngle");
  }

  public static class Spindexer {
    public static final LoggedTunableNumber FEED_VOLTS =
        new LoggedTunableNumber("Setpoints/Spindexer/FeedVolts", 0.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Spindexer/ExhaustVolts", 0.0);
  }

  public static class Intake {
    public static final LoggedTunableNumber TUCK_ANGLE =
        new LoggedTunableNumber("Setpoints/Intake/TuckAngle", 0.0);
    public static final LoggedTunableNumber EXTEND_ANGLE =
        new LoggedTunableNumber("Setpoints/Intake/ExtendAngle", 0.0);

    public static final LoggedTunableNumber INTAKE_VOLTS =
        new LoggedTunableNumber("Setpoints/Intake/IntakeVolts", 0.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Intake/ExhaustVolts", 0.0);
  }

  public static class Loader {
    public static final LoggedTunableNumber FEED_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/FeedVolts", 0.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/ExhaustVolts", 0.0);
    public static final LoggedTunableNumber SLOW_EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/SlowExhaustVolts", 0.0);
  }

  public static class Shooter {
    public static final LoggedTunableNumber CLOSE_HUB_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/CloseHubSpeed");
  }
}

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

  public static class Hopper {
    public static final LoggedTunableNumber FEED_DUTYCYCLE =
        new LoggedTunableNumber("Setpoints/Hopper/FeedDutyCycle", 0.0);
    public static final LoggedTunableNumber EXHAUST_DUTYCYCLE =
        new LoggedTunableNumber("Setpoints/Hopper/ExhaustDutyCycle", 0.0);
  }

  public static class Intake {
    public static final LoggedTunableNumber TUCK_ANGLE =
        new LoggedTunableNumber("Setpoints/Intake/TuckAngle", 0.0);
    public static final LoggedTunableNumber EXTEND_ANGLE =
        new LoggedTunableNumber("Setpoints/Intake/ExtendAngle", 0.0);

    public static final LoggedTunableNumber INTAKE_DUTYCYCLE =
        new LoggedTunableNumber("Setpoints/Intake/IntakeDutyCycle", 0.0);
    public static final LoggedTunableNumber EXHAUST_DUTYCYCLE =
        new LoggedTunableNumber("Setpoints/Intake/ExhaustDutyCycle", 0.0);
  }

  public static class Loader {
    public static final LoggedTunableNumber FEED_DUTYCYCLE =
        new LoggedTunableNumber("Setpoints/Loader/FeedDutyCycle", 0.0);
    public static final LoggedTunableNumber EXHAUST_DUTYCYCLE =
        new LoggedTunableNumber("Setpoints/Loader/ExhaustDutyCycle", 0.0);
    public static final LoggedTunableNumber SLOW_EXHAUST_DUTYCYCLE =
        new LoggedTunableNumber("Setpoints/Loader/SlowExhaustDutyCycle", 0.0);
  }

  public static class Shooter {
    public static final LoggedTunableNumber CLOSE_HUB_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/CloseHubSpeed");
  }
}

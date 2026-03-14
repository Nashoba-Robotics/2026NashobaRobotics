package frc.robot.autos;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public final class AutoConstants {
  public static final Distance kAutoLinearEpsilon = Units.Centimeters.of(4.0);
  public static final Angle kAutoAngleEpsilon = Units.Degrees.of(1.0);
  public static final Time kDelayTime = Units.Milliseconds.of(80);
  public static final Time kDefaultTrajectoryTimeout = Units.Seconds.of(1.0);
}

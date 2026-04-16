package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public final class AutoConstants {
  public static final Distance kAutoLinearEpsilon = Units.Centimeters.of(4.0);
  public static final Angle kAutoAngleEpsilon = Units.Degrees.of(5.0);
  public static final Time kDelayTime = Units.Milliseconds.of(80);
  public static final Time kDefaultTrajectoryTimeout = Units.Seconds.of(10.0);

  public static final Rotation2d beachAngleThreshold = Rotation2d.fromDegrees(10.0);

  public static final Distance kBumpLinearEpsilon = Units.Centimeters.of(20.0);
  public static final Angle kBumpAngleEpsilon = Units.Degree.of(15.0);

  public static final Time kShootingTime = Units.Seconds.of(2.85);
  public static final Time kDelayIntakeRetract = Units.Seconds.of(1.5);
}

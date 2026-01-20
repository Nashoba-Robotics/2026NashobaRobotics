package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class ShootingCalculator {

  private static final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopTime));
  private static final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopTime));

  private static double driveAngleRads = Double.NaN;
  private static double hoodAngleRads = Double.NaN;
  private static double shooterSpeedRadsPerSec;
  private static double lastDriveAngleRads;
  private static double lastHoodAngleRads;
  private static double driveVelocityRadsPerSec;
  private static double hoodVelocityRadsPerSec;

  public record ShooterSetpoint(
      double driveAngleRads,
      double hoodAngleRads,
      double driveVelocityRadsPerSec,
      double hoodVelocityRadsPerSec,
      double shooterSpeedRadsPerSec) {}

  private static final InterpolatingDoubleTreeMap distanceHoodAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceShooterVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    distanceHoodAngleMap.put(0.0, 0.0);
    distanceHoodAngleMap.put(0.0, 0.0);
    distanceHoodAngleMap.put(0.0, 0.0);

    distanceShooterVelocityMap.put(0.0, 0.0);
    distanceShooterVelocityMap.put(0.0, 0.0);
    distanceShooterVelocityMap.put(0.0, 0.0);

    distanceTimeOfFlightMap.put(0.0, 0.0);
    distanceTimeOfFlightMap.put(0.0, 0.0);
    distanceTimeOfFlightMap.put(0.0, 0.0);
  }

  public static ShooterSetpoint makeSetpoint(Drive drive, Pose2d target) {

    double robotToTargetDistance =
        target.getTranslation().getDistance(drive.getPose().getTranslation());

    ChassisSpeeds fieldRelativeVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());

    // TODO: Maybe iterate over timeOfFlight to find new time of flight with futurePose
    double timeOfFlight = distanceTimeOfFlightMap.get(robotToTargetDistance);
    Pose2d futurePose =
        new Pose2d(
            drive.getPose().getX() + fieldRelativeVelocity.vxMetersPerSecond * timeOfFlight,
            drive.getPose().getY() + fieldRelativeVelocity.vyMetersPerSecond * timeOfFlight,
            drive.getPose().getRotation());
    double futurePosetoTargetDistance =
        target.getTranslation().getDistance(drive.getPose().getTranslation());

    driveAngleRads =
        target.getTranslation().minus(futurePose.getTranslation()).getAngle().getRadians();
    hoodAngleRads = distanceHoodAngleMap.get(futurePosetoTargetDistance);
    shooterSpeedRadsPerSec = distanceShooterVelocityMap.get(futurePosetoTargetDistance);

    if (Double.isNaN(lastDriveAngleRads)) lastDriveAngleRads = driveAngleRads;
    if (Double.isNaN(lastHoodAngleRads)) lastHoodAngleRads = hoodAngleRads;

    driveVelocityRadsPerSec =
        driveAngleFilter.calculate((driveAngleRads - lastDriveAngleRads) / Constants.loopTime);
    hoodVelocityRadsPerSec =
        hoodAngleFilter.calculate((hoodAngleRads - lastHoodAngleRads) / Constants.loopTime);

    lastDriveAngleRads = driveAngleRads;
    lastHoodAngleRads = hoodAngleRads;

    Logger.recordOutput("targetPose", target);
    Logger.recordOutput("futureRobotPose", futurePose);

    return new ShooterSetpoint(
        driveAngleRads,
        hoodAngleRads,
        driveVelocityRadsPerSec,
        hoodVelocityRadsPerSec,
        shooterSpeedRadsPerSec);
  }
}

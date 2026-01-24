package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class ShootingUtil {

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

    distanceTimeOfFlightMap.put(0.0, 1.0);
    distanceTimeOfFlightMap.put(2.5, 1.075);
    distanceTimeOfFlightMap.put(5.0, 1.15);
  }

  public static ShooterSetpoint makeSetpoint(Drive drive, Pose2d target) {

    ChassisSpeeds fieldRelativeVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());

    double timeOfFlight;
    Pose2d futurePose = drive.getPose();
    double futurePosetoTargetDistance =
        target.getTranslation().getDistance(drive.getPose().getTranslation());

    // iterate over timeOfFlight for each new future pose because it would be slightly different
    for (int i = 0; i < 5; i++) {
      timeOfFlight = distanceTimeOfFlightMap.get(futurePosetoTargetDistance);
      futurePose =
          new Pose2d(
              drive.getPose().getX() + fieldRelativeVelocity.vxMetersPerSecond * timeOfFlight,
              drive.getPose().getY() + fieldRelativeVelocity.vyMetersPerSecond * timeOfFlight,
              drive.getPose().getRotation());
      futurePosetoTargetDistance = target.getTranslation().getDistance(futurePose.getTranslation());
    }

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

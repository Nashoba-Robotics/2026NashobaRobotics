package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class ShootingUtil {

  private static final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopTime));
  private static final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopTime));

  private static ShooterSetpoint setpoint = null;

  private static double lastHubDriveAngleRads;
  private static double lastHubHoodAngleRads;

  private static Translation2d robotToShooterOffset =
      new Translation2d(Units.inchesToMeters(7.0), Units.inchesToMeters(0.0));

  public record ShooterSetpoint(
      boolean isShuttling,
      Rotation2d driveAngleRads,
      double hoodAngleRads,
      Rotation2d driveVelocityRadsPerSec,
      double hoodVelocityRadsPerSec,
      double shooterSpeedRadsPerSec) {}

  private static final InterpolatingTreeMap<Double, Rotation2d> hubDistanceHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap hubDistanceShooterVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hubDistanceTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingTreeMap<Double, Rotation2d> shuttleDistanceHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shuttleDistanceShooterVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shuttleDistanceTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    hubDistanceHoodAngleMap.put(0.0, Rotation2d.fromDegrees(0.0));
    hubDistanceHoodAngleMap.put(1.49, Rotation2d.fromDegrees(7.5));
    hubDistanceHoodAngleMap.put(2.00, Rotation2d.fromDegrees(13.0));
    hubDistanceHoodAngleMap.put(2.50, Rotation2d.fromDegrees(15.5));
    hubDistanceHoodAngleMap.put(3.00, Rotation2d.fromDegrees(19.5));
    hubDistanceHoodAngleMap.put(3.25, Rotation2d.fromDegrees(21.5));
    hubDistanceHoodAngleMap.put(3.51, Rotation2d.fromDegrees(23.0));
    hubDistanceHoodAngleMap.put(3.99, Rotation2d.fromDegrees(23.5));
    hubDistanceHoodAngleMap.put(4.5, Rotation2d.fromDegrees(24.5));

    hubDistanceShooterVelocityMap.put(0.00, 232.5);
    hubDistanceShooterVelocityMap.put(1.49, 232.5);
    hubDistanceShooterVelocityMap.put(2.00, 232.5);
    hubDistanceShooterVelocityMap.put(2.25, 250.0);
    hubDistanceShooterVelocityMap.put(2.50, 245.0);
    hubDistanceShooterVelocityMap.put(3.00, 262.0);
    hubDistanceShooterVelocityMap.put(3.51, 267.5);
    hubDistanceShooterVelocityMap.put(3.99, 270.0);
    hubDistanceShooterVelocityMap.put(4.5, 295.0);

    hubDistanceTimeOfFlightMap.put(0.0, 1.05);
    hubDistanceTimeOfFlightMap.put(1.0, 1.05);
    hubDistanceTimeOfFlightMap.put(3.0, 1.075);
    hubDistanceTimeOfFlightMap.put(5.0, 1.125);

    shuttleDistanceHoodAngleMap.put(0.0, Rotation2d.fromDegrees(37.0));
    shuttleDistanceHoodAngleMap.put(2.5, Rotation2d.fromDegrees(37.0));
    // shuttleDistanceHoodAngleMap.put(3.5, Rotation2d.fromDegrees(35.0));
    // shuttleDistanceHoodAngleMap.put(4.5, Rotation2d.fromDegrees(35.0));

    shuttleDistanceShooterVelocityMap.put(0.0, 215.0);
    shuttleDistanceShooterVelocityMap.put(1.5, 215.0);
    shuttleDistanceShooterVelocityMap.put(2.5, 295.0);
    shuttleDistanceShooterVelocityMap.put(6.5, 380.0);

    shuttleDistanceTimeOfFlightMap.put(0.0, 1.00);
    shuttleDistanceTimeOfFlightMap.put(3.0, 1.10);
    shuttleDistanceTimeOfFlightMap.put(5.0, 1.5);
    shuttleDistanceTimeOfFlightMap.put(10.0, 2.0);
  }

  public static ShooterSetpoint makeSetpoint(Drive drive) {
    if (setpoint != null) return setpoint;

    boolean isShuttling =
        AllianceFlipUtil.applyX(drive.getPose().getX()) >= FieldConstants.LinesVertical.hubCenter;

    Translation2d target =
        isShuttling
            ? getShuttleTargetPose(drive.getPose())
            : AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());

    double driveAngleRads = Double.NaN;
    double hoodAngleRads = Double.NaN;
    double shooterSpeedRadsPerSec;
    double driveVelocityRadsPerSec;
    double hoodVelocityRadsPerSec;

    ChassisSpeeds fieldRelativeRobotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
    // ChassisSpeeds fieldRelativeShooterVelocity =
    //     GeomUtil.transformVelocity(
    //         fieldRelativeRobotVelocity, robotToShooterOffset, drive.getRotation());

    double timeOfFlight;
    Pose2d futurePose = drive.getPose();
    double futurePosetoTargetDistance = target.getDistance(drive.getPose().getTranslation());

    // iterate over timeOfFlight for each new future pose because it would be slightly different
    for (int i = 0; i < 25; i++) {
      timeOfFlight =
          isShuttling
              ? shuttleDistanceTimeOfFlightMap.get(futurePosetoTargetDistance)
              : hubDistanceTimeOfFlightMap.get(futurePosetoTargetDistance);
      futurePose =
          new Pose2d(
              drive.getPose().getX() + fieldRelativeRobotVelocity.vxMetersPerSecond * timeOfFlight,
              drive.getPose().getY() + fieldRelativeRobotVelocity.vyMetersPerSecond * timeOfFlight,
              drive.getPose().getRotation());
      futurePosetoTargetDistance = target.getDistance(futurePose.getTranslation());
    }

    driveAngleRads = target.minus(futurePose.getTranslation()).getAngle().getRadians();
    hoodAngleRads =
        isShuttling
            ? shuttleDistanceHoodAngleMap.get(futurePosetoTargetDistance).getRadians()
            : hubDistanceHoodAngleMap.get(futurePosetoTargetDistance).getRadians();
    shooterSpeedRadsPerSec =
        isShuttling
            ? shuttleDistanceShooterVelocityMap.get(futurePosetoTargetDistance)
            : hubDistanceShooterVelocityMap.get(futurePosetoTargetDistance);

    if (Double.isNaN(lastHubDriveAngleRads)) lastHubDriveAngleRads = driveAngleRads;
    if (Double.isNaN(lastHubHoodAngleRads)) lastHubHoodAngleRads = hoodAngleRads;

    // drive angular speed wraparound
    double deltaAngleRads = driveAngleRads - lastHubDriveAngleRads;
    if (deltaAngleRads > Math.PI) deltaAngleRads -= (2 * Math.PI);
    else if (deltaAngleRads < -Math.PI) deltaAngleRads += (2 * Math.PI);

    // stops rapid switches in futurePose target velocity when near the goal
    driveVelocityRadsPerSec =
        (futurePosetoTargetDistance >= 1.0)
            ? driveAngleFilter.calculate(
                MathUtil.clamp(
                    (deltaAngleRads) / Constants.loopTime,
                    -drive.getMaxAngularSpeedRadPerSec(),
                    drive.getMaxAngularSpeedRadPerSec()))
            : 0;
    hoodVelocityRadsPerSec =
        hoodAngleFilter.calculate((hoodAngleRads - lastHubHoodAngleRads) / Constants.loopTime);

    lastHubDriveAngleRads = driveAngleRads;
    lastHubHoodAngleRads = hoodAngleRads;

    Logger.recordOutput("ShootingUtil/TargetPose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("ShootingUtil/FutureRobotPose", futurePose);
    Logger.recordOutput("ShootingUtil/FuturePosetoTargetDistance", futurePosetoTargetDistance);

    setpoint =
        new ShooterSetpoint(
            isShuttling,
            Rotation2d.fromRadians(driveAngleRads),
            hoodAngleRads,
            Rotation2d.fromRadians(driveVelocityRadsPerSec),
            hoodVelocityRadsPerSec,
            shooterSpeedRadsPerSec);

    return setpoint;
  }

  public static void clearShooterSetpoint() {
    setpoint = null;
  }

  public static Translation2d getShuttleTargetPose(Pose2d drivePose) {
    return AllianceFlipUtil.apply(
        (AllianceFlipUtil.applyY(drivePose.getY()) <= FieldConstants.fieldWidth / 2)
            ? new Translation2d(4.041, 2.498)
            : new Translation2d(4.041, 5.545));
  }
}

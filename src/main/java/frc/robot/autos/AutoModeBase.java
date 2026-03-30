package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Stopwatch;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class AutoModeBase {
  private static AutoRoutine routine;
  private static Stopwatch stopwatch = new Stopwatch();

  public AutoModeBase(AutoFactory factory, String name) {
    routine = factory.newRoutine(name);
  }

  /**
   * @return Trajectory from choreo
   */
  public AutoTrajectory trajectory(String name) {
    return routine.trajectory(name);
  }

  public AutoTrajectory trajectory(String name, int index) {
    return routine.trajectory(name, index);
  }

  /**
   * Runs an accuracy-based command for choreo following
   *
   * @param trajectory
   * @param timeout
   */
  public static Command cmdWithRotationAccuracy(
      Drive drive, AutoTrajectory trajectory, Time timeout) {
    return Commands.defer(
            () ->
                new FunctionalCommand(
                    trajectory.cmd()::initialize,
                    trajectory.cmd()::execute,
                    trajectory.cmd()::end,
                    () -> rotationIsFinished(drive, trajectory)),
            Set.of(drive))
        .withTimeout(trajectory.getRawTrajectory().getTotalTime() + timeout.in(Units.Seconds));
  }

  public static Command cmdWithRotationAccuracy(Drive drive, AutoTrajectory trajectory) {
    return cmdWithRotationAccuracy(drive, trajectory, AutoConstants.kDefaultTrajectoryTimeout);
  }

  /**
   * Runs an accuracy-based command for choreo following
   *
   * @param trajectory
   * @param timeout
   */
  public static Command cmdWithAccuracy(
      Drive drive, AutoTrajectory trajectory, Time timeout, Distance epsilonDist) {
    return Commands.defer(
            () ->
                new FunctionalCommand(
                    trajectory.cmd()::initialize,
                    trajectory.cmd()::execute,
                    trajectory.cmd()::end,
                    () -> isFinished(drive, trajectory, epsilonDist)),
            Set.of(drive))
        .withTimeout(trajectory.getRawTrajectory().getTotalTime() + timeout.in(Units.Seconds));
  }

  /**
   * Returns an accuracy-based command for choreo following, including the default timeout
   *
   * @param trajectory
   */
  public static Command cmdWithAccuracy(
      Drive drive, AutoTrajectory trajectory, Distance epsilonDist) {
    return cmdWithAccuracy(drive, trajectory, AutoConstants.kDefaultTrajectoryTimeout, epsilonDist);
  }

  public static Command cmdWithAccuracy(Drive drive, AutoTrajectory trajectory) {
    return cmdWithAccuracy(drive, trajectory, AutoConstants.kAutoLinearEpsilon);
  }

  public static Command cmdWithInterrupt(Drive drive, AutoTrajectory trajectory) {
    return trajectory
        .cmd()
        .handleInterrupt(
            () -> {
              drive.runVelocity(new ChassisSpeeds());
              Logger.recordOutput(
                  "Choreo/Auto Trajectory Command Interrupted", Timer.getFPGATimestamp());
            });
  }

  private static boolean rotationIsFinished(Drive drive, AutoTrajectory trajectory) {
    Pose2d currentPose = drive.getPose();
    Pose2d finalPose = trajectory.getFinalPose().get();
    Angle epsilonAngle = AutoConstants.kAutoAngleEpsilon;

    return MathUtil.angleModulus(
            Math.abs(currentPose.getRotation().minus(finalPose.getRotation()).getRadians()))
        < epsilonAngle.in(Units.Radians);
  }

  private static boolean translationIsFinished(
      Drive drive, AutoTrajectory trajectory, Distance epsilonDist) {
    Pose2d currentPose = drive.getPose();
    Pose2d finalPose = trajectory.getFinalPose().get();

    Logger.recordOutput(
        "Choreo/Distance Away Inches",
        currentPose.getTranslation().getDistance(finalPose.getTranslation()) * 39.37);

    return currentPose.getTranslation().getDistance(finalPose.getTranslation())
        < epsilonDist.in(Units.Meters);
  }

  private static boolean isFinished(Drive drive, AutoTrajectory trajectory, Distance epsilonDist) {
    boolean translationCompleted = translationIsFinished(drive, trajectory, epsilonDist);
    boolean rotationCompleted = rotationIsFinished(drive, trajectory);

    Logger.recordOutput("Choreo/Translation Completed", translationCompleted);
    Logger.recordOutput("Choreo/Rotation Completed", rotationCompleted);

    if (translationCompleted && rotationCompleted) {
      stopwatch.startIfNotRunning();
      if (stopwatch.getTime().gte(AutoConstants.kDelayTime)) {
        stopwatch.reset();
        return true;
      }
    } else if (!translationCompleted || !rotationCompleted) {
      stopwatch.reset();
    }

    Logger.recordOutput("Choreo/Stopwatch Time", stopwatch.getTimeAsDouble());
    return false;
  }

  public static Command antiBeach(Drive drive) {
    return Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, -1.5, 0)), drive)
        .until(() -> !drive.isBeached());
  }

  public void newRoutine(Command... sequence) {
    routine
        .active()
        .onTrue(Commands.sequence(sequence).withName("Auto Routine Sequential Command Group"));
  }

  public void atTranslation(
      String eventName, Command event, Distance epsilon, AutoTrajectory... trajectories) {
    for (AutoTrajectory trajectory : trajectories) {
      trajectory.atTranslation(eventName, epsilon.in(Units.Meters)).onTrue(event);
    }
  }

  public void atTranslation(String eventName, Command event, AutoTrajectory... trajectories) {
    atTranslation(eventName, event, AutoConstants.kAutoLinearEpsilon, trajectories);
  }

  public AutoRoutine getRoutine() {
    return routine;
  }

  public Command asCommand() {
    return routine.cmd();
  }
}

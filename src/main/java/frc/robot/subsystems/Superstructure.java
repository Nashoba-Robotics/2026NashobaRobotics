package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.Presets;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.loader.Loader;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShootingUtil;
import frc.robot.util.ShootingUtil.ShooterSetpoint;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Climber climber;
  private final Hood hood;
  private final Hopper hopper;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;
  private final Loader loader;
  private final Shooter shooter;

  private ShooterSetpoint hubShootingSetpoint;

  public Superstructure(
      Drive drive,
      Climber climber,
      Hood hood,
      Hopper hopper,
      IntakeDeploy intakeDeploy,
      IntakeRoller intakeRoller,
      Loader loader,
      Shooter shooter) {
    this.drive = drive;
    this.climber = climber;
    this.hood = hood;
    this.hopper = hopper;
    this.intakeDeploy = intakeDeploy;
    this.intakeRoller = intakeRoller;
    this.loader = loader;
    this.shooter = shooter;

    hood.setDefaultCommand(hood.runPositionCommand(Presets.Hood.TUCK_ANGLE.get()));
  }

  @Override
  public void periodic() {
    hubShootingSetpoint = ShootingUtil.makeSetpoint(drive, FieldConstants.getAllianceHubPose2d());

    Logger.recordOutput("DriveCommands/atAngleSetpoint", DriveCommands.atAngleSetpoint());
    Logger.recordOutput(
        "DriveCommands/atDriveToPoseSetpoint", DriveCommands.atDriveToPoseSetpoint());
  }

  public Command aimAtHubCommand(DoubleSupplier driveXSupplier, DoubleSupplier driveYSupplier) {
    return new ParallelCommandGroup(
        DriveCommands.joystickDriveAtAngle(
            drive,
            driveXSupplier,
            driveYSupplier,
            this::getHubShootingSetpointDriveAngle,
            this::getHubShootingSetpointDriveVelocity),
        hood.runTrackedPositionCommand(
            this::getHubShootingSetpointHoodAngle, this::getHubShootingSetpointHoodVelocity),
        shooter.runTrackedVelocityCommand(this::getHubShootingSetpointShooterSpeed));
  }

  public Command shootCommand() {
    return new ParallelCommandGroup(
        hopper.runDutyCycleCommand(Presets.Hopper.FEED_DUTYCYCLE),
        loader.runDutyCycleCommand(Presets.Loader.FEED_DUTYCYCLE));
  }

  public Command endShootCommand() {
    return new SequentialCommandGroup(
        loader.runDutyCycleCommand(Presets.Loader.SLOW_EXHAUST_DUTYCYCLE).withTimeout(0.1),
        new ParallelCommandGroup(hopper.stopCommand(), loader.stopCommand()));
  }

  public Command stopAllRollersCommand() {
    return new ParallelCommandGroup(
        intakeRoller.stopCommand(),
        hopper.stopCommand(),
        loader.stopCommand(),
        shooter.stopCommand());
  }

  public Rotation2d getHubShootingSetpointDriveAngle() {
    return Rotation2d.fromRadians(hubShootingSetpoint.driveAngleRads());
  }

  public Rotation2d getHubShootingSetpointDriveVelocity() {
    return Rotation2d.fromRadians(hubShootingSetpoint.driveVelocityRadsPerSec());
  }

  public double getHubShootingSetpointHoodAngle() {
    return hubShootingSetpoint.hoodAngleRads();
  }

  public double getHubShootingSetpointHoodVelocity() {
    return hubShootingSetpoint.hoodVelocityRadsPerSec();
  }

  public double getHubShootingSetpointShooterSpeed() {
    return hubShootingSetpoint.shooterSpeedRadsPerSec();
  }
}

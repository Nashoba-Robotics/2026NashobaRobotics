package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Presets;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.loader.Loader;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.ShootingUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Hood hood;
  private final Spindexer spindexer;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;
  private final Loader loader;
  private final Shooter leftShooter;
  private final Shooter rightShooter;

  public Superstructure(
      Drive drive,
      Hood hood,
      Spindexer spindexer,
      IntakeDeploy intakeDeploy,
      IntakeRoller intakeRoller,
      Loader loader,
      Shooter leftShooter,
      Shooter rightShooter) {
    this.drive = drive;
    this.hood = hood;
    this.spindexer = spindexer;
    this.intakeDeploy = intakeDeploy;
    this.intakeRoller = intakeRoller;
    this.loader = loader;
    this.leftShooter = leftShooter;
    this.rightShooter = rightShooter;

    hood.setDefaultCommand(
        hood.runPositionCommand(Units.degreesToRadians(Presets.Hood.TUCK_ANGLE_DEG.get())));
    leftShooter.setDefaultCommand(leftShooter.stopCommand());
    rightShooter.setDefaultCommand(rightShooter.stopCommand());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("DriveCommands/atAngleSetpoint", DriveCommands.atAngleSetpoint());
  }

  public Command aimCommand(DoubleSupplier driveXSupplier, DoubleSupplier driveYSupplier) {
    return new ParallelCommandGroup(
        DriveCommands.joystickDriveAtAngle(
            drive,
            driveXSupplier,
            driveYSupplier,
            () -> ShootingUtil.makeSetpoint(drive).driveAngleRads(),
            () -> ShootingUtil.makeSetpoint(drive).driveVelocityRadsPerSec()),
        hood.runTrackedPositionCommand(
            () -> ShootingUtil.makeSetpoint(drive).hoodAngleRads(),
            () -> ShootingUtil.makeSetpoint(drive).hoodVelocityRadsPerSec()),
        leftShooter.runTrackedVelocityCommand(
            () -> ShootingUtil.makeSetpoint(drive).shooterSpeedRadsPerSec()),
        rightShooter.runTrackedVelocityCommand(
            () -> ShootingUtil.makeSetpoint(drive).shooterSpeedRadsPerSec()));
  }

  public Command shootCommand() {
    return new ParallelCommandGroup(
        spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
        loader.runVoltageCommand(Presets.Loader.FEED_VOLTS));
  }

  public Command endShootCommand() {
    return new ParallelCommandGroup(
        spindexer.stopCommand(),
        new SequentialCommandGroup(
            loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS).withTimeout(0.25),
            loader.stopCommand()));
  }

  public Command deployIntake() {
    return new SequentialCommandGroup(
        intakeDeploy
            .runVoltageCommand(() -> 4.0)
            .until(() -> intakeDeploy.getPosition() >= Units.degreesToRadians(110)),
        intakeDeploy.runVoltageCommand(() -> 0.30));
  }

  public Command retractIntake() {
    return new ParallelCommandGroup(
        new SequentialCommandGroup(
            intakeDeploy
                .runVoltageCommand(() -> -4.0)
                .until(() -> intakeDeploy.getPosition() <= Units.degreesToRadians(10)),
            intakeDeploy.runVoltageCommand(() -> -0.10)),
        intakeRoller.runVoltageCommand(Presets.Intake.SLOW_INTAKE_VOLTS).withTimeout(1.0));
  }

  public Command autoDeployIntake() {
    return new SequentialCommandGroup(
        intakeDeploy
            .runVoltageCommand(() -> 4.0)
            .until(() -> intakeDeploy.getPosition() >= Units.degreesToRadians(110)),
        intakeDeploy.runVoltageCommand(() -> 0.40));
  }

  public Command autoRetractIntake() {
    return new ParallelCommandGroup(
            new SequentialCommandGroup(
                intakeDeploy
                    .runVoltageCommand(() -> -4.0)
                    .until(() -> intakeDeploy.getPosition() <= Units.degreesToRadians(10)),
                intakeDeploy.runVoltageCommand(() -> -0.10)),
            intakeRoller.runVoltageCommand(Presets.Intake.SLOW_INTAKE_VOLTS).withTimeout(1.0))
        .until(() -> intakeDeploy.getPosition() <= 5);
  }

  public Command stopAllRollersCommand() {
    return new ParallelCommandGroup(
        intakeRoller.stopCommand(),
        spindexer.stopCommand(),
        loader.stopCommand(),
        leftShooter.stopCommand(),
        rightShooter.stopCommand());
  }

  public Command autoShoot() {
    return new ParallelCommandGroup(
            aimCommand(() -> 0.0, () -> 0.0),
            new SequentialCommandGroup(
                new WaitUntilCommand(
                    () ->
                        leftShooter.atSetpoint()
                            && rightShooter.atSetpoint()
                            && hood.atSetpoint()
                            && DriveCommands.atAngleSetpoint()),
                new ParallelCommandGroup(
                    loader.runVoltageCommand(Presets.Loader.FEED_VOLTS),
                    spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS))))
        .withTimeout(3.5)
        .andThen(autoEndShootCommand());
  }

  public Command autoEndShootCommand() {
    return new ParallelCommandGroup(
        spindexer.stopCommand(),
        new SequentialCommandGroup(
            loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS).withTimeout(0.5),
            loader.stopCommand()),
        leftShooter.stopCommand(),
        rightShooter.stopCommand(),
        hood.runPositionCommand(Units.degreesToRadians(Presets.Hood.TUCK_ANGLE_DEG.get())));
  }
}

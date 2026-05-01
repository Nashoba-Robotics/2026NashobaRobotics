package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Presets;
import frc.robot.autos.AutoConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.entryroller.EntryRoller;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.rollerfloor.RollerFloor;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShootingUtil;
import java.util.function.DoubleSupplier;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Hood hood;
  private final RollerFloor rollerFloor;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;
  private final EntryRoller entryRoller;
  private final Shooter shooter;

  public Superstructure(
      Drive drive,
      Hood hood,
      RollerFloor rollerFloor,
      IntakeDeploy intakeDeploy,
      IntakeRoller intakeRoller,
      EntryRoller entryRoller,
      Shooter shooter) {
    this.drive = drive;
    this.hood = hood;
    this.rollerFloor = rollerFloor;
    this.intakeDeploy = intakeDeploy;
    this.intakeRoller = intakeRoller;
    this.entryRoller = entryRoller;
    this.shooter = shooter;

    hood.setDefaultCommand(
        hood.runPositionCommand(Units.degreesToRadians(Presets.Hood.TUCK_ANGLE_DEG.get())));
    shooter.setDefaultCommand(shooter.stopCommand());
    entryRoller.setDefaultCommand(entryRoller.stopCommand());
    rollerFloor.setDefaultCommand(rollerFloor.stopCommand());
    intakeRoller.setDefaultCommand(intakeRoller.stopCommand());
  }

  @Override
  public void periodic() {
    ShootingUtil.makeSetpoint(drive);
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
        shooter.runTrackedVelocityCommand(
            () -> ShootingUtil.makeSetpoint(drive).shooterSpeedRadsPerSec()));
  }

  public Command shootCommand() {
    return new ParallelCommandGroup(
        rollerFloor.runVelocityCommand(Presets.RollerFloor.FEED_SPEED),
        entryRoller.runVelocityCommand(Presets.EntryRoller.FEED_SPEED));
  }

  public Command endShootCommand() {
    return new ParallelCommandGroup(
        rollerFloor.stopCommand(),
        new SequentialCommandGroup(
            entryRoller.runVelocityCommand(Presets.EntryRoller.EXHAUST_SPEED).withTimeout(0.25),
            entryRoller.stopCommand()));
  }

  public Command deployIntake() {
    return new SequentialCommandGroup(
        intakeDeploy
            .runVoltageCommand(() -> 8.0)
            .until(() -> intakeDeploy.getPosition() >= Units.degreesToRadians(130)),
        intakeDeploy.runVoltageCommand(() -> 0.30));
  }

  public Command retractIntake() {
    return new ParallelCommandGroup(
        new SequentialCommandGroup(
            intakeDeploy
                .runVoltageCommand(() -> -8.0)
                .until(() -> intakeDeploy.getPosition() <= Units.degreesToRadians(25)),
            intakeDeploy.runVoltageCommand(() -> -0.30)),
        intakeRoller.runVoltageCommand(Presets.Intake.SLOW_INTAKE_VOLTS).withTimeout(1.0));
  }

  public Command autoRunIntake() {
    return deployIntake()
        .alongWith(intakeRoller.runVelocityCommand(Presets.Intake.AUTO_INTAKE_SPEED));
  }

  public Command autoRetractIntake() {
    return retractIntake().until(() -> intakeDeploy.getPosition() <= Units.degreesToRadians(25.0));
  }

  public Command autoShakeIntake() {
    return new SequentialCommandGroup(
            autoRetractIntake().withTimeout(0.25), deployIntake().withTimeout(0.25))
        .repeatedly()
        .withTimeout(2.25)
        .andThen(autoRetractIntake());
  }

  public Command stopAllRollersCommand() {
    return new ParallelCommandGroup(
        intakeRoller.stopCommand(),
        rollerFloor.stopCommand(),
        entryRoller.stopCommand(),
        shooter.stopCommand());
  }

  public Command autoShoot() {
    return new ParallelCommandGroup(
            aimCommand(() -> 0.0, () -> 0.0),
            new SequentialCommandGroup(
                new SequentialCommandGroup(
                    Commands.waitSeconds(0.01), Commands.waitUntil(this::inShootingTolerance)),
                new ParallelCommandGroup(
                    entryRoller.runVelocityCommand(Presets.EntryRoller.FEED_SPEED),
                    rollerFloor.runVelocityCommand(Presets.RollerFloor.FEED_SPEED))))
        .withTimeout(AutoConstants.kShootingTime)
        .andThen(autoEndShootCommand());
  }

  public Command autoEndShootCommand() {
    return new ParallelCommandGroup(
        rollerFloor.stopCommand(),
        new SequentialCommandGroup(
            entryRoller.runVelocityCommand(Presets.EntryRoller.EXHAUST_SPEED).withTimeout(0.5),
            entryRoller.stopCommand()),
        shooter.stopCommand(),
        hood.runPositionCommand(Units.degreesToRadians(Presets.Hood.TUCK_ANGLE_DEG.get())));
  }

  public boolean inShootingTolerance() {
    return hood.atSetpoint()
        && (ShootingUtil.makeSetpoint(drive).isShuttling()
            ? shooter.inTolerance(Constants.Shooter.SHUTTLE_VELOCITY_TOLERANCE.get())
            : shooter.atSetpoint())
        && DriveCommands.atShootingSetpoint(drive);
  }
}

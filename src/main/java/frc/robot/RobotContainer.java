package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.LeftT_2NZSafe_Auto;
import frc.robot.autos.LeftT_2NZSteal_Auto;
import frc.robot.autos.RightT_2NZSafe_Auto;
import frc.robot.autos.RightT_2NZSteal_Auto;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.generated.TunerConstants;
import frc.robot.subsystems.entryroller.EntryRoller;
import frc.robot.subsystems.entryroller.EntryRollerIO;
import frc.robot.subsystems.entryroller.EntryRollerIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakedeploy.IntakeDeployIO;
import frc.robot.subsystems.intakedeploy.IntakeDeployIOTalonFX;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.intakeroller.IntakeRollerIO;
import frc.robot.subsystems.intakeroller.IntakeRollerIOTalonFX;
import frc.robot.subsystems.rollerfloor.RollerFloor;
import frc.robot.subsystems.rollerfloor.RollerFloorIO;
import frc.robot.subsystems.rollerfloor.RollerFloorIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Hood hood;
  private final RollerFloor rollerFloor;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;
  private final EntryRoller entryRoller;
  private final Shooter shooter;
  private final Superstructure superstructure;
  private final LEDSubsystem leds = new LEDSubsystem();

  private final AutoFactory autoFactory;

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2));

        // climber = new Climber(new ClimberIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        rollerFloor = new RollerFloor(new RollerFloorIOTalonFX());
        intakeDeploy = new IntakeDeploy(new IntakeDeployIOTalonFX());
        intakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX());
        entryRoller = new EntryRoller(new EntryRollerIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());

        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        hood = new Hood(new HoodIO() {});
        rollerFloor = new RollerFloor(new RollerFloorIO() {});
        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        entryRoller = new EntryRoller(new EntryRollerIO() {});
        shooter = new Shooter(new ShooterIO() {});

        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        hood = new Hood(new HoodIO() {});
        rollerFloor = new RollerFloor(new RollerFloorIO() {});
        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        entryRoller = new EntryRoller(new EntryRollerIO() {});
        shooter = new Shooter(new ShooterIO() {});

        break;
    }

    superstructure =
        new Superstructure(
            drive, hood, rollerFloor, intakeDeploy, intakeRoller, entryRoller, shooter);

    autoFactory = drive.getAutoFactory();

    NamedCommands.registerCommand("shoot", superstructure.autoShoot());
    NamedCommands.registerCommand(
        "intakeRoller", intakeRoller.runVoltageCommand(Presets.Intake.INTAKE_VOLTS));
    NamedCommands.registerCommand("intakeDeploy", superstructure.deployIntake());
    NamedCommands.registerCommand("intakeRetract", superstructure.autoRetractIntake());
    NamedCommands.registerCommand(
        "tuckHood",
        hood.runPositionCommand(Units.degreesToRadians(Presets.Hood.TUCK_ANGLE_DEG.get())));

    SmartDashboard.putData(
        "RunEverythingForTuning",
        new ParallelCommandGroup(
            entryRoller.runVelocityCommand(Presets.EntryRoller.TUNING_SPEED),
            rollerFloor.runVelocityCommand(Presets.RollerFloor.TUNING_SPEED),
            intakeRoller.runVoltageCommand(Presets.Intake.TUNING_VOLTS),
            intakeDeploy.runTrackedPositionCommand(
                () -> Units.degreesToRadians(Presets.Intake.TUNING_ANGLE_DEG.getAsDouble())),
            shooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
            hood.runTrackedPositionCommand(
                () -> Units.degreesToRadians(Presets.Hood.TUNING_ANGLE_DEG.get()), () -> 0.0)));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

    autoChooser.addOption("PP Right T-2NZ", new PathPlannerAuto("T-2NZ-No Climb", false));
    autoChooser.addOption("PP Left T-2NZ", new PathPlannerAuto("T-2NZ-No Climb", true));
    autoChooser.addOption("PP Right B-Outpost-Depot", new PathPlannerAuto("B-Outpost-Depot-Climb"));
    autoChooser.addOption("dumbShoot", superstructure.autoShoot().withTimeout(7.0));

    autoChooser.addOption(
        "Choreo Right T-2NZSteal",
        new RightT_2NZSteal_Auto(drive, superstructure, autoFactory).asCommand());
    autoChooser.addOption(
        "Choreo Right T-2NZSafe",
        new RightT_2NZSafe_Auto(drive, superstructure, autoFactory).asCommand());
    autoChooser.addOption(
        "Choreo Left T-2NZSteal",
        new LeftT_2NZSteal_Auto(drive, superstructure, autoFactory).asCommand());
    autoChooser.addOption(
        "Choreo Left T-2NZSafe",
        new LeftT_2NZSafe_Auto(drive, superstructure, autoFactory).asCommand());

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    driver
        .start()
        .and(driver.back())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    Trigger inShootingTolerance =
        new Trigger(
            () ->
                hood.atSetpoint()
                    && shooter.atSetpoint()
                    && DriveCommands.atShootingSetpoint(drive));

    // Shoot bindings
    driver
        .rightTrigger()
        .whileTrue(superstructure.aimCommand(() -> -driver.getLeftY(), () -> -driver.getLeftX()))
        .and(inShootingTolerance.debounce(0.15, DebounceType.kFalling))
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    // Force shoot
    driver
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                rollerFloor.runVelocityCommand(Presets.RollerFloor.FEED_SPEED),
                entryRoller.runVelocityCommand(Presets.EntryRoller.FEED_SPEED)));

    // Close shot fallback
    driver
        .b()
        .whileTrue(
            new ParallelCommandGroup(
                rollerFloor.runVelocityCommand(Presets.RollerFloor.FEED_SPEED),
                entryRoller.runVelocityCommand(Presets.EntryRoller.FEED_SPEED),
                hood.runPositionCommand(
                    Units.degreesToRadians(Presets.Hood.CLOSE_HUB_ANGLE_DEG.getAsDouble())),
                shooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED.getAsDouble())));

    // Intake deploy and retract
    driver.leftTrigger().onTrue(superstructure.deployIntake());
    driver.leftTrigger().whileTrue(intakeRoller.runVoltageCommand(Presets.Intake.INTAKE_VOLTS));

    driver.leftBumper().onTrue(superstructure.retractIntake());

    // Manual spit and feed
    driver.x().whileTrue(intakeRoller.runVoltageCommand(Presets.Intake.EXHAUST_VOLTS));
    driver
        .y()
        .whileTrue(
            rollerFloor
                .runVelocityCommand(Presets.RollerFloor.EXHAUST_SPEED)
                .alongWith(entryRoller.runVelocityCommand(Presets.EntryRoller.EXHAUST_SPEED)));

    driver.a().whileTrue(intakeRoller.runVoltageCommand(() -> 12.0));

    // Tune shot
    // driver
    //     .b()
    //     .whileTrue(
    //         new ParallelCommandGroup(
    //             DriveCommands.joystickDriveAtAngle(
    //                 drive,
    //                 () -> 0.0,
    //                 () -> 0.0,
    //                 () -> ShootingUtil.makeSetpoint(drive).driveAngleRads(),
    //                 () -> ShootingUtil.makeSetpoint(drive).driveVelocityRadsPerSec()),
    //             shooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
    //             hood.runTrackedPositionCommand(
    //                 () -> Units.degreesToRadians(Presets.Hood.TUNING_ANGLE_DEG.get()), () ->
    // 0.0),
    //             rollerFloor.runVelocityCommand(Presets.RollerFloor.FEED_SPEED.getAsDouble()),
    //             entryRoller.runVelocityCommand(Presets.EntryRoller.FEED_SPEED.getAsDouble())));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

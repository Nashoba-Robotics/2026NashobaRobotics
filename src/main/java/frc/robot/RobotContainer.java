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
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.generated.TunerConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakedeploy.IntakeDeployIO;
import frc.robot.subsystems.intakedeploy.IntakeDeployIOTalonFX;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.intakeroller.IntakeRollerIO;
import frc.robot.subsystems.intakeroller.IntakeRollerIOTalonFX;
import frc.robot.subsystems.loader.Loader;
import frc.robot.subsystems.loader.LoaderIO;
import frc.robot.subsystems.loader.LoaderIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOTalonFX;
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
  private final Spindexer spindexer;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;
  private final Loader loader;
  private final Shooter leftShooter;
  private final Shooter rightShooter;
  private final Superstructure superstructure;

  //   private final LEDSubsystem leds = new LEDSubsystem();

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
        spindexer = new Spindexer(new SpindexerIOTalonFX());
        intakeDeploy = new IntakeDeploy(new IntakeDeployIOTalonFX());
        intakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX());
        loader = new Loader(new LoaderIOTalonFX());
        leftShooter =
            new Shooter(
                new ShooterIOTalonFX(
                    true,
                    Constants.Shooter.LEFT_SHOOTER_LEADER_ID,
                    Constants.Shooter.LEFT_SHOOTER_FOLLOWER_ID),
                true);
        rightShooter =
            new Shooter(
                new ShooterIOTalonFX(
                    false,
                    Constants.Shooter.RIGHT_SHOOTER_LEADER_ID,
                    Constants.Shooter.RIGHT_SHOOTER_FOLLOWER_ID),
                false);

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
        spindexer = new Spindexer(new SpindexerIO() {});
        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        loader = new Loader(new LoaderIO() {});
        leftShooter = new Shooter(new ShooterIO() {}, true);
        rightShooter = new Shooter(new ShooterIO() {}, false);

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
        spindexer = new Spindexer(new SpindexerIO() {});
        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        loader = new Loader(new LoaderIO() {});
        leftShooter = new Shooter(new ShooterIO() {}, true);
        rightShooter = new Shooter(new ShooterIO() {}, false);

        break;
    }

    superstructure =
        new Superstructure(
            drive, hood, spindexer, intakeDeploy, intakeRoller, loader, leftShooter, rightShooter);

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
            loader.runVoltageCommand(Presets.Loader.TUNING_VOLTS),
            spindexer.runVoltageCommand(Presets.Spindexer.TUNING_VOLTS),
            intakeRoller.runVoltageCommand(Presets.Intake.TUNING_VOLTS),
            intakeDeploy.runTrackedPositionCommand(
                () -> Units.degreesToRadians(Presets.Intake.TUNING_ANGLE_DEG.getAsDouble())),
            leftShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
            rightShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
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
                    && leftShooter.atSetpoint()
                    && rightShooter.atSetpoint()
                    && DriveCommands.atAngleSetpoint());

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
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS)));

    // Close shot fallback
    driver
        .b()
        .whileTrue(
            new ParallelCommandGroup(
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS),
                hood.runPositionCommand(
                    Units.degreesToRadians(Presets.Hood.CLOSE_HUB_ANGLE_DEG.getAsDouble())),
                leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED.getAsDouble()),
                rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED.getAsDouble())));

    // Intake deploy and retract
    driver.leftTrigger().onTrue(superstructure.deployIntake());
    driver.leftTrigger().whileTrue(intakeRoller.runVoltageCommand(Presets.Intake.INTAKE_VOLTS));

    driver.leftBumper().onTrue(superstructure.retractIntake());

    // Manual spit and feed
    driver.x().whileTrue(intakeRoller.runVoltageCommand(Presets.Intake.EXHAUST_VOLTS));
    driver
        .y()
        .whileTrue(
            spindexer
                .runVoltageCommand(Presets.Spindexer.EXHAUST_VOLTS)
                .alongWith(loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS)));

    driver.a().whileTrue(intakeRoller.runVoltageCommand(() -> 12.0));

    // Tune shot
    // driver
    //     .b()
    //     .whileTrue(
    //         new ParallelCommandGroup(
    //             leftShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
    //             rightShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
    //             hood.runTrackedPositionCommand(
    //                 () -> Units.degreesToRadians(Presets.Hood.TUNING_ANGLE_DEG.get()), () ->
    // 0.0),
    //             spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
    //             loader.runVoltageCommand(Presets.Loader.FEED_VOLTS)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

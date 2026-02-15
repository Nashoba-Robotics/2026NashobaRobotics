package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
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
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Climber climber;
  private final Hood hood;
  private final Spindexer spindexer;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;
  private final Loader loader;
  private final Shooter leftShooter;
  private final Shooter rightShooter;
  private final Superstructure superstructure;

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

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

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision(camera0Name, robotToCamera0),
        //         new VisionIOPhotonVision(camera1Name, robotToCamera1));

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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        climber = new Climber(new ClimberIO() {});

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

        climber = new Climber(new ClimberIO() {});
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

        climber = new Climber(new ClimberIO() {});
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
            drive,
            climber,
            hood,
            spindexer,
            intakeDeploy,
            intakeRoller,
            loader,
            leftShooter,
            rightShooter);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

    SmartDashboard.putData(
        "RunEverythingForTuning",
        new ParallelCommandGroup(
            loader.runVoltageCommand(Presets.Loader.TUNING_VOLTS),
            spindexer.runVoltageCommand(Presets.Spindexer.TUNING_VOLTS),
            intakeRoller.runVoltageCommand(Presets.Intake.TUNING_VOLTS),
            intakeDeploy.runTrackedPositionCommand(Presets.Intake.TUNING_ANGLE_DEG),
            leftShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
            rightShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
            hood.runTrackedPositionCommand(
                () -> Units.degreesToRadians(Presets.Hood.TUNING_ANGLE_DEG.get()), () -> 0.0)));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    Trigger inAllianceZone =
        new Trigger(
            () -> {
              Pose2d robotPose = AllianceFlipUtil.apply(drive.getPose());
              return (robotPose.getX() <= FieldConstants.LinesVertical.allianceZone + 0.40);
            });

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

    // driver
    //     .rightTrigger()
    //     .and(inAllianceZone)
    //     .whileTrue(superstructure.hubAimCommand(() -> -driver.getLeftY(), () ->
    // -driver.getLeftX()))
    //     .and(hood::atSetpoint)
    //     .and(leftShooter::atSetpoint)
    //     .and(rightShooter::atSetpoint)
    //     .and(DriveCommands::atAngleSetpoint)
    //     .whileTrue(superstructure.shootCommand())
    //     .onFalse(superstructure.endShootCommand());
    // driver
    //     .rightTrigger()
    //     .and(inAllianceZone.negate())
    //     .whileTrue(
    //         superstructure.shuttleAimCommand(() -> -driver.getLeftY(), () -> -driver.getLeftX()))
    //     .and(hood::atSetpoint)
    //     .and(leftShooter::atSetpoint)
    //     .and(rightShooter::atSetpoint)
    //     .and(DriveCommands::atAngleSetpoint)
    //     .whileTrue(superstructure.shootCommand())
    //     .onFalse(superstructure.endShootCommand());

    driver
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS),
                hood.runPositionCommand(
                    Units.degreesToRadians(Presets.Hood.CLOSE_HUB_ANGLE_DEG.getAsDouble())),
                leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED.getAsDouble()),
                rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED.getAsDouble())))
        .onFalse(new ParallelCommandGroup(leftShooter.stopCommand(), rightShooter.stopCommand()));

    driver.leftBumper().whileTrue(intakeRoller.runVoltageCommand(Presets.Intake.INTAKE_VOLTS));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

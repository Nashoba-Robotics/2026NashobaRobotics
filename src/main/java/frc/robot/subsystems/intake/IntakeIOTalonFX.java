package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX deploy;
  private final TalonFX roller;
  private final TalonFXConfiguration deployConfig;
  private final TalonFXConfiguration rollerConfig;

  private final CANcoder encoder;
  private final CANcoderConfiguration encoderConfig;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  private final MotionMagicDutyCycle positionDutyCycle = new MotionMagicDutyCycle(0);
  private final VelocityDutyCycle velocityDutyCylcle = new VelocityDutyCycle(0);

  public IntakeIOTalonFX() {
    deploy = new TalonFX(Constants.Intake.DEPLOY_MOTOR_ID);
    roller = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);
    deployConfig = new TalonFXConfiguration();
    rollerConfig = new TalonFXConfiguration();

    encoder = new CANcoder(Constants.Intake.DEPLOY_ENCODER_ID);
    encoderConfig = new CANcoderConfiguration();

    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.ROLLER_STATOR_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.ROLLER_STATOR_LIMIT;

    rollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rollerConfig.Feedback.SensorToMechanismRatio = Constants.Hopper.GEAR_RATIO;

    rollerConfig.MotorOutput.Inverted = Constants.Intake.ROLLER_INVERTED;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    roller.getConfigurator().apply(rollerConfig);

    deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    deployConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.DEPLOY_STATOR_LIMIT;
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.DEPLOY_SUPPLY_LIMIT;

    deployConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    deployConfig.Feedback.FeedbackRemoteSensorID = Constants.Intake.DEPLOY_ENCODER_ID;
    deployConfig.Feedback.RotorToSensorRatio = Constants.Intake.GEAR_RATIO;

    deployConfig.MotorOutput.Inverted = Constants.Intake.DEPLOY_INVERTED;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    deployConfig.Slot0 = Constants.Intake.PID;

    deploy.getConfigurator().apply(deployConfig);

    encoderConfig.MagnetSensor.SensorDirection = Constants.Intake.ENCODER_DIRECTION;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Intake.ENCODER_OFFSET;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        Constants.Intake.ENCODER_DISCONTINUITY_POINT;

    encoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerConnected = roller.isConnected();
    inputs.rollerTempCelsius = roller.getDeviceTemp().getValueAsDouble();
    inputs.rollerVelocityRadPerSec =
        Units.rotationsToRadians(roller.getVelocity().getValueAsDouble());
    inputs.rollerAppliedVolts = roller.getMotorVoltage().getValueAsDouble();
    inputs.rollerStatorCurrentAmps = roller.getStatorCurrent().getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = roller.getSupplyCurrent().getValueAsDouble();

    inputs.deployConnected = deploy.isConnected();
    inputs.deployTempCelsius = deploy.getDeviceTemp().getValueAsDouble();
    inputs.deployPositionRad = Units.rotationsToRadians(deploy.getPosition().getValueAsDouble());
    inputs.deployPositionSetpointRad = Units.rotationsToRadians(deploy.getClosedLoopOutput().getValueAsDouble());
    inputs.deployVelocityRadPerSec =
        Units.rotationsToRadians(deploy.getVelocity().getValueAsDouble());
    inputs.deployAppliedVolts = deploy.getMotorVoltage().getValueAsDouble();
    inputs.deployStatorCurrentAmps = deploy.getStatorCurrent().getValueAsDouble();
    inputs.deploySupplyCurrentAmps = deploy.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runRollerDutyCycle(double percent) {
    roller.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void runDeployDutyCycle(double percent) {
    deploy.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void runDeployPosition(double positionRad) {
    deploy.setControl(positionDutyCycle.withPosition(positionRad));
  }
  
  @Override
  public void rollerStop() {
    roller.setControl(new NeutralOut());
  }

  @Override
  public void deployStop() {
    deploy.setControl(new NeutralOut());
  }

  @Override
  public void deploySetPID(double kP, double kD) {
    deployConfig.Slot0.kP = kP;
    deployConfig.Slot0.kD = kD;
    deploy.getConfigurator().apply(deployConfig);
  }
}

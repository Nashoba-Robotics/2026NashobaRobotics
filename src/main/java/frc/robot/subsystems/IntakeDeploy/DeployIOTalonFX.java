package frc.robot.subsystems.IntakeDeploy;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DeployIOTalonFX implements DeployIO {

  private final TalonFX roller;
  private final TalonFX deploy;
  private final TalonFXConfiguration deployConfig;
  private final TalonFXConfiguration rollerConfig;

  private final CANcoder encoder;
  private final CANcoderConfiguration encoderConfig;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withEnableFOC(true);

  public DeployIOTalonFX() {
    roller = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);
    deploy = new TalonFX(Constants.Intake.DEPLOY_MOTOR_ID);

    rollerConfig = new TalonFXConfiguration();
    deployConfig = new TalonFXConfiguration();

    encoder = new CANcoder(Constants.Intake.DEPLOY_ENCODER_ID);
    encoderConfig = new CANcoderConfiguration();

    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.ROLLER_STATOR_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.ROLLER_STATOR_LIMIT;

    rollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rollerConfig.Feedback.SensorToMechanismRatio =
        Constants.Intake.ROLLER_SENSOR_TO_MECHANISM_GEAR_RATIO;

    rollerConfig.MotorOutput.Inverted = Constants.Intake.ROLLER_INVERTED;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    roller.getConfigurator().apply(rollerConfig);

    deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    deployConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.DEPLOY_STATOR_LIMIT;
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.DEPLOY_SUPPLY_LIMIT;

    deployConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    deployConfig.Feedback.FeedbackRemoteSensorID = Constants.Intake.DEPLOY_ENCODER_ID;
    deployConfig.Feedback.RotorToSensorRatio =
        Constants.Intake.DEPLOY_ROTOR_TO_MECHANISM_GEAR_RATIO;
    deployConfig.Feedback.SensorToMechanismRatio =
        Constants.Intake.DEPLOY_SENSOR_TO_MECHANISM_GEAR_RATIO;

    deployConfig.MotorOutput.Inverted = Constants.Intake.DEPLOY_INVERTED;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    deployConfig.Slot0.kP = Constants.Intake.kP;
    deployConfig.Slot0.kD = Constants.Intake.kD;
    deployConfig.Slot0.kS = Constants.Intake.kS;
    deployConfig.Slot0.kV = Constants.Intake.kV;
    deployConfig.Slot0.kA = Constants.Intake.kA;

    deploy.getConfigurator().apply(deployConfig);

    encoderConfig.MagnetSensor.SensorDirection = Constants.Intake.ENCODER_DIRECTION;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Intake.ENCODER_OFFSET;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        Constants.Intake.ENCODER_DISCONTINUITY_POINT;

    encoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void updateInputs(DeployIOInputs inputs) {
    inputs.deployConnected = deploy.isConnected();
    inputs.deployTempCelsius = deploy.getDeviceTemp().getValueAsDouble();
    inputs.deployAbsolutePositionRads = Units.rotationsToRadians(encoder.getPosition().getValueAsDouble());
    inputs.deployRotorPositionRads = Units.rotationsToRadians(deploy.getPosition().getValueAsDouble());
    inputs.deployPositionSetpointRads =
        Units.rotationsToRadians(deploy.getClosedLoopOutput().getValueAsDouble());
    inputs.deployVelocityRadsPerSec =
        Units.rotationsToRadians(deploy.getVelocity().getValueAsDouble());
    inputs.deployAppliedVolts = deploy.getMotorVoltage().getValueAsDouble();
    inputs.deployStatorCurrentAmps = deploy.getStatorCurrent().getValueAsDouble();
    inputs.deploySupplyCurrentAmps = deploy.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runDeployDutyCycle(double percent) {
    deploy.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void runDeployPosition(double positionRads) {
    deploy.setControl(positionVoltage.withPosition(Units.radiansToRotations(positionRads)));
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

  @Override
  public void deploySetFeedForward(double kS, double kG, double kV, double kA) {
    deployConfig.Slot0.kS = kS;
    deployConfig.Slot0.kG = kG;
    deployConfig.Slot0.kV = kV;
    deployConfig.Slot0.kA = kA;
    deploy.getConfigurator().apply(deployConfig);
  }
}

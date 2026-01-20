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

public class IntakeDeployIOTalonFX implements IntakeDeployIO {

  private final TalonFX roller;
  private final TalonFX deploy;
  private final TalonFXConfiguration deployConfig;
  private final TalonFXConfiguration rollerConfig;

  private final CANcoder encoder;
  private final CANcoderConfiguration encoderConfig;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withEnableFOC(true);

  public IntakeDeployIOTalonFX() {
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
  public void updateInputs(IntakeDeployIOInputs inputs) {
    inputs.connected = deploy.isConnected();
    inputs.tempCelsius = deploy.getDeviceTemp().getValueAsDouble();
    inputs.absolutePositionRads =
        Units.rotationsToRadians(encoder.getPosition().getValueAsDouble());
    inputs.rotorPositionRads = Units.rotationsToRadians(deploy.getPosition().getValueAsDouble());
    inputs.positionSetpointRads =
        Units.rotationsToRadians(deploy.getClosedLoopOutput().getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(deploy.getVelocity().getValueAsDouble());
    inputs.appliedVolts = deploy.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentAmps = deploy.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = deploy.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runDutyCycle(double percent) {
    deploy.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void runPosition(double positionRads) {
    deploy.setControl(positionVoltage.withPosition(Units.radiansToRotations(positionRads)));
  }

  @Override
  public void stop() {
    deploy.setControl(new NeutralOut());
  }

  @Override
  public void setPID(double kP, double kD) {
    deployConfig.Slot0.kP = kP;
    deployConfig.Slot0.kD = kD;
    deploy.getConfigurator().apply(deployConfig);
  }

  @Override
  public void setFeedForward(double kS, double kG, double kV, double kA) {
    deployConfig.Slot0.kS = kS;
    deployConfig.Slot0.kG = kG;
    deployConfig.Slot0.kV = kV;
    deployConfig.Slot0.kA = kA;
    deploy.getConfigurator().apply(deployConfig);
  }
}

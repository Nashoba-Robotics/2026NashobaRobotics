package frc.robot.subsystems.intakeDeploy;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class IntakeDeployIOTalonFX implements IntakeDeployIO {

  private final TalonFX deploy;
  private final TalonFXConfiguration deployConfig;

  private final CANcoder encoder;
  private final CANcoderConfiguration encoderConfig;

  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Angle> absolutePosition;
  private final StatusSignal<Angle> rotorPosition;
  private final StatusSignal<Double> positionSetpoint;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withEnableFOC(true);

  public IntakeDeployIOTalonFX() {
    deploy = new TalonFX(Constants.Intake.DEPLOY_MOTOR_ID);
    deployConfig = new TalonFXConfiguration();

    encoder = new CANcoder(Constants.Intake.DEPLOY_ENCODER_ID);
    encoderConfig = new CANcoderConfiguration();

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

    encoderConfig.MagnetSensor.SensorDirection = Constants.Intake.ENCODER_DIRECTION;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Intake.ENCODER_OFFSET;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        Constants.Intake.ENCODER_DISCONTINUITY_POINT;

    PhoenixUtil.tryUntilOk(5, () -> deploy.getConfigurator().apply(deployConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig, 0.25));

    temp = deploy.getDeviceTemp();
    absolutePosition = encoder.getAbsolutePosition();
    rotorPosition = deploy.getPosition();
    positionSetpoint = deploy.getClosedLoopReference();
    velocity = deploy.getVelocity();
    appliedVolts = deploy.getMotorVoltage();
    statorCurrent = deploy.getStatorCurrent();
    supplyCurrent = deploy.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / Constants.loopTime,
        temp,
        absolutePosition,
        rotorPosition,
        positionSetpoint,
        velocity,
        appliedVolts,
        statorCurrent,
        supplyCurrent);
    deploy.optimizeBusUtilization();
    encoder.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        false,
        temp,
        absolutePosition,
        rotorPosition,
        positionSetpoint,
        velocity,
        appliedVolts,
        statorCurrent,
        supplyCurrent);
  }

  @Override
  public void updateInputs(IntakeDeployIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        temp,
        absolutePosition,
        rotorPosition,
        positionSetpoint,
        velocity,
        appliedVolts,
        statorCurrent,
        supplyCurrent);

    inputs.motorConnected =
        BaseStatusSignal.isAllGood(
            temp,
            rotorPosition,
            positionSetpoint,
            velocity,
            appliedVolts,
            statorCurrent,
            supplyCurrent);
    inputs.encoderConnected = BaseStatusSignal.isAllGood(absolutePosition);
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.absolutePositionRads = Units.rotationsToRadians(absolutePosition.getValueAsDouble());
    inputs.rotorPositionRads = Units.rotationsToRadians(rotorPosition.getValueAsDouble());
    inputs.positionSetpointRads = Units.rotationsToRadians(positionSetpoint.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
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

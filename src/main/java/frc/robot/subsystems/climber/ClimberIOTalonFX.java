package frc.robot.subsystems.climber;

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

public class ClimberIOTalonFX implements ClimberIO {

  private final TalonFX climber;

  private final TalonFXConfiguration config;

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

  public ClimberIOTalonFX() {
    climber = new TalonFX(Constants.Climber.MOTOR_ID);
    config = new TalonFXConfiguration();

    encoder = new CANcoder(Constants.Climber.ENCODER_ID);
    encoderConfig = new CANcoderConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Climber.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Climber.SUPPLY_LIMIT;

    config.MotorOutput.Inverted = Constants.Climber.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Climber.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.Climber.MOTION_MAGIC_ACCELERATION;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.Climber.FORWARD_SOFT_LIMIT.getRotations();
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.Climber.REVERSE_SOFT_LIMIT.getRotations();

    config.Feedback.FeedbackRemoteSensorID = Constants.Climber.ENCODER_ID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = Constants.Climber.ROTOR_TO_MECHANISM_GEAR_RATIO;
    config.Feedback.SensorToMechanismRatio = Constants.Climber.SENSOR_TO_MECHANISM_GEAR_RATIO;

    config.Slot0.kP = Constants.Climber.kP.get();
    config.Slot0.kD = Constants.Climber.kD.get();
    config.Slot0.kS = Constants.Climber.kS.get();
    config.Slot0.kG = Constants.Climber.kG.get();
    config.Slot0.kV = Constants.Climber.kV.get();
    config.Slot0.kA = Constants.Climber.kA.get();

    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        Constants.Climber.ENCODER_DISCONTINUITY_POINT;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Climber.ENCODER_OFFSET;
    encoderConfig.MagnetSensor.SensorDirection = Constants.Climber.ENCODER_DIRECTION;

    PhoenixUtil.tryUntilOk(5, () -> climber.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig, 0.25));

    temp = climber.getDeviceTemp();
    absolutePosition = encoder.getAbsolutePosition();
    rotorPosition = climber.getPosition();
    positionSetpoint = climber.getClosedLoopReference();
    velocity = climber.getVelocity();
    appliedVolts = climber.getMotorVoltage();
    statorCurrent = climber.getStatorCurrent();
    supplyCurrent = climber.getSupplyCurrent();

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
    climber.optimizeBusUtilization();
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
  public void updateInputs(ClimberIOInputs inputs) {
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
    climber.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void runPosition(double positionRads) {
    climber.setControl(positionVoltage.withPosition(Units.radiansToRotations(positionRads)));
  }

  @Override
  public void stop() {
    climber.setControl(new NeutralOut());
  }

  @Override
  public void setPID(double kP, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    climber.getConfigurator().apply(config);
  }

  @Override
  public void setFeedForward(double kS, double kG, double kV, double kA) {
    config.Slot0.kS = kS;
    config.Slot0.kG = kG;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    climber.getConfigurator().apply(config);
  }
}

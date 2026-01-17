package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {

  private final TalonFX climber;
  private final TalonFXConfiguration config;

  private final CANcoder encoder;
  private final CANcoderConfiguration encoderConfig;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  private final MotionMagicDutyCycle positionDutyCycle = new MotionMagicDutyCycle(0);

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

    config.Slot0 = Constants.Climber.PID;

    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        Constants.Climber.ENCODER_DISCONTINUITY_POINT;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Climber.ENCODER_OFFSET;
    encoderConfig.MagnetSensor.SensorDirection = Constants.Climber.ENCODER_DIRECTION;

    climber.getConfigurator().apply(config);
    encoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.connected = climber.isConnected();
    inputs.tempCelsius = climber.getDeviceTemp().getValueAsDouble();
    inputs.velocityRadPerSec = Units.rotationsToRadians(climber.getVelocity().getValueAsDouble());
    inputs.absolutePositionRad =
        Units.rotationsToRadians(encoder.getPosition().getValueAsDouble());
    inputs.rotorPositionRad = Units.rotationsToRadians(climber.getPosition().getValueAsDouble());
    inputs.positionSetpointRad = Units.rotationsToRadians(climber.getClosedLoopReference().getValueAsDouble());
    inputs.appliedVolts = climber.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentAmps = climber.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = climber.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runDutyCycle(double percent) {
    climber.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void runPosition(double positionRad) {
    climber.setControl(positionDutyCycle.withPosition(Units.radiansToRotations(positionRad)));
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
}

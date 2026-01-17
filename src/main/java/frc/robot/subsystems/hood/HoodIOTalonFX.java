package frc.robot.subsystems.hood;

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

public class HoodIOTalonFX implements HoodIO {

  private final TalonFX hood;
  private final TalonFXConfiguration config;

  private final CANcoder encoder;
  private final CANcoderConfiguration encoderConfig;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  private final MotionMagicDutyCycle positionDutyCycle = new MotionMagicDutyCycle(0);

  public HoodIOTalonFX() {
    hood = new TalonFX(Constants.Hood.MOTOR_ID);
    config = new TalonFXConfiguration();

    encoder = new CANcoder(Constants.Hood.ENCODER_ID);
    encoderConfig = new CANcoderConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Hood.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.SUPPLY_LIMIT;

    config.Feedback.FeedbackRemoteSensorID = Constants.Hood.ENCODER_ID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = Constants.Hood.ROTOR_TO_MECHANISM_GEAR_RATIO;
    config.Feedback.SensorToMechanismRatio = Constants.Hood.SENSOR_TO_MECHANISM_GEAR_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = Constants.Hood.INVERTED;

    config.Slot0 = Constants.Hood.PID;

    encoderConfig.MagnetSensor.SensorDirection = Constants.Hood.ENCODER_DIRECTION;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        Constants.Hood.ENCODER_DISCONTINUITY_POINT;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Hood.ENCODER_OFFSET;

    hood.getConfigurator().apply(config);
    encoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.connected = hood.isConnected();
    inputs.tempCelsius = hood.getDeviceTemp().getValueAsDouble();
    inputs.absolutePositionRad =
        Units.rotationsToRadians(encoder.getPosition().getValueAsDouble());
    inputs.rotorPositionRad = Units.rotationsToRadians(hood.getPosition().getValueAsDouble());
    inputs.positionSetpointRad = Units.rotationsToRadians(hood.getClosedLoopReference().getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(hood.getVelocity().getValueAsDouble());
    inputs.appliedVolts = hood.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentAmps = hood.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = hood.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runDutyCycle(double percent) {
    hood.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void runPosition(double positionRad) {
    hood.setControl(positionDutyCycle.withPosition(positionRad));
  }

  @Override
  public void stop() {
    hood.setControl(new NeutralOut());
  }

  @Override
  public void setPID(double kP, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    hood.getConfigurator().apply(config);
  }
}

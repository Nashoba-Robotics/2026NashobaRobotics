package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX implements HoodIO {

  private final TalonFX hood;
  private final TalonFXConfiguration motorConfig;

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

  private final LoggedTunableNumber minAngleDeg = new LoggedTunableNumber("Hood/minAngleDeg", 0.0);
  private final LoggedTunableNumber maxAngleDeg = new LoggedTunableNumber("Hood/maxAngleDeg", 43.0);
  
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);

  public HoodIOTalonFX() {
    hood = new TalonFX(Constants.Hood.MOTOR_ID, Constants.Hood.CANBUS);
    motorConfig = new TalonFXConfiguration();

    encoder = new CANcoder(Constants.Hood.ENCODER_ID, Constants.Hood.CANBUS);
    encoderConfig = new CANcoderConfiguration();

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = Constants.Hood.STATOR_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Hood.SUPPLY_LIMIT;

    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = Constants.Hood.ENCODER_ID;
    motorConfig.Feedback.RotorToSensorRatio =
        Constants.Hood.ROTOR_TO_SENSOR_GEAR_RATIO;
    motorConfig.Feedback.SensorToMechanismRatio =
        Constants.Hood.SENSOR_TO_MECHANISM_GEAR_RATIO;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = Constants.Hood.INVERTED;

    motorConfig.Slot0.kP = Constants.Hood.kP.get();
    motorConfig.Slot0.kD = Constants.Hood.kD.get();
    motorConfig.Slot0.kS = Constants.Hood.kS.get();
    motorConfig.Slot0.kV = Constants.Hood.kV.get();
    motorConfig.Slot0.kA = Constants.Hood.kA.get();
    motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    encoderConfig.MagnetSensor.SensorDirection = Constants.Hood.ENCODER_DIRECTION;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Hood.ENCODER_OFFSET;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        Constants.Hood.ENCODER_DISCONTINUITY_POINT;

    PhoenixUtil.tryUntilOk(5, () -> hood.getConfigurator().apply(motorConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig, 0.25));

    temp = hood.getDeviceTemp();
    absolutePosition = encoder.getAbsolutePosition();
    rotorPosition = hood.getPosition();
    positionSetpoint = hood.getClosedLoopReference();
    velocity = hood.getVelocity();
    appliedVolts = hood.getMotorVoltage();
    statorCurrent = hood.getStatorCurrent();
    supplyCurrent = hood.getSupplyCurrent();

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
    hood.optimizeBusUtilization();

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
  public void updateInputs(HoodIOInputs inputs) {
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
    inputs.encoderConnected =
        BaseStatusSignal.isAllGood(absolutePosition);
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
  public void runVoltage(double volts) {
    hood.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runPosition(double positionRads, double velocityRadsPerSec) {
    hood.setControl(
        positionTorqueCurrentFOC
            .withPosition(
                Units.radiansToRotations(
                    MathUtil.clamp(
                        positionRads,
                        Units.degreesToRadians(minAngleDeg.get()),
                        Units.degreesToRadians(maxAngleDeg.get()))))
            .withVelocity(Units.radiansToRotations(velocityRadsPerSec)));
  }

  @Override
  public void stop() {
    hood.setControl(new NeutralOut());
  }

  @Override
  public void setPID(double kP, double kD) {
    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kD = kD;
    hood.getConfigurator().apply(motorConfig);
  }

  @Override
  public void setFeedForward(double kS, double kG, double kV, double kA) {
    motorConfig.Slot0.kS = kS;
    motorConfig.Slot0.kG = kG;
    motorConfig.Slot0.kV = kV;
    motorConfig.Slot0.kA = kA;
    hood.getConfigurator().apply(motorConfig);
  }
}

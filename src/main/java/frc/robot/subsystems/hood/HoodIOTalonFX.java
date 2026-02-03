package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

public class HoodIOTalonFX implements HoodIO {

  private final TalonFX hood;
  private final TalonFXConfiguration config;

  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Angle> rotorPosition;
  private final StatusSignal<Double> positionSetpoint;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionVoltage = new PositionVoltage(0).withEnableFOC(true);

  public HoodIOTalonFX() {
    hood = new TalonFX(Constants.Hood.MOTOR_ID);
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Hood.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.SUPPLY_LIMIT;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Hood.GEAR_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = Constants.Hood.INVERTED;

    config.Slot0.kP = Constants.Hood.kP.get();
    config.Slot0.kD = Constants.Hood.kD.get();
    config.Slot0.kS = Constants.Hood.kS.get();
    config.Slot0.kV = Constants.Hood.kV.get();
    config.Slot0.kA = Constants.Hood.kA.get();

    PhoenixUtil.tryUntilOk(5, () -> hood.getConfigurator().apply(config, 0.25));

    temp = hood.getDeviceTemp();
    rotorPosition = hood.getPosition();
    positionSetpoint = hood.getClosedLoopReference();
    velocity = hood.getVelocity();
    appliedVolts = hood.getMotorVoltage();
    statorCurrent = hood.getStatorCurrent();
    supplyCurrent = hood.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / Constants.loopTime,
        temp,
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
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.rotorPositionRads = Units.rotationsToRadians(rotorPosition.getValueAsDouble());
    inputs.positionSetpointRads = Units.rotationsToRadians(positionSetpoint.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
  }

  @Override
  public void runVoltage(double percent) {
    hood.setControl(voltageOut.withOutput(percent));
  }

  @Override
  public void runPosition(double positionRads, double velocityRadsPerSec) {
    hood.setControl(
        positionVoltage
            .withPosition(Units.radiansToRotations(positionRads))
            .withVelocity(Units.radiansToRotations(velocityRadsPerSec)));
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

  @Override
  public void setFeedForward(double kS, double kG, double kV, double kA) {
    config.Slot0.kS = kS;
    config.Slot0.kG = kG;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    hood.getConfigurator().apply(config);
  }
}

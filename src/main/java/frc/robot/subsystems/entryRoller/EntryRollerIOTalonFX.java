package frc.robot.subsystems.entryRoller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class EntryRollerIOTalonFX implements EntryRollerIO {

  private final TalonFX entryRoller;
  private final TalonFXConfiguration config;

  private final StatusSignal<Temperature> temp;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(false);

  public EntryRollerIOTalonFX() {
    entryRoller = new TalonFX(Constants.EntryRoller.MOTOR_ID, Constants.EntryRoller.CANBUS);
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.EntryRoller.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.EntryRoller.SUPPLY_LIMIT;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.EntryRoller.GEAR_RATIO;

    config.MotorOutput.Inverted = Constants.EntryRoller.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(5, () -> entryRoller.getConfigurator().apply(config));

    temp = entryRoller.getDeviceTemp();
    velocity = entryRoller.getVelocity();
    appliedVolts = entryRoller.getMotorVoltage();
    statorCurrent = entryRoller.getStatorCurrent();
    supplyCurrent = entryRoller.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / Constants.loopTime, temp, velocity, appliedVolts, statorCurrent, supplyCurrent);

    entryRoller.optimizeBusUtilization();

    PhoenixUtil.registerSignals(false, temp, velocity, appliedVolts, statorCurrent, supplyCurrent);
  }

  @Override
  public void updateInputs(EntryRollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(temp, velocity, appliedVolts, statorCurrent, supplyCurrent);

    inputs.connected =
        BaseStatusSignal.isAllGood(temp, velocity, appliedVolts, statorCurrent, supplyCurrent);
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
  }

  @Override
  public void runVoltage(double volts) {
    entryRoller.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    entryRoller.setControl(new NeutralOut());
  }
}

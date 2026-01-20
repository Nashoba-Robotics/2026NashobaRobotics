package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
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

public class HopperIOTalonFX implements HopperIO {

  private final TalonFX hopper;
  private final TalonFXConfiguration config;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0).withEnableFOC(true);

  private final StatusSignal<Temperature> temp;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  public HopperIOTalonFX() {
    hopper = new TalonFX(Constants.Hopper.MOTOR_ID);
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Hopper.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Hopper.SUPPLY_LIMIT;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Hopper.GEAR_RATIO;

    config.MotorOutput.Inverted = Constants.Hopper.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(5, () -> hopper.getConfigurator().apply(config));

    temp = hopper.getDeviceTemp();
    velocity = hopper.getVelocity();
    appliedVolts = hopper.getMotorVoltage();
    statorCurrent = hopper.getStatorCurrent();
    supplyCurrent = hopper.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / Constants.loopTime, temp, velocity, appliedVolts, statorCurrent, supplyCurrent);

    hopper.optimizeBusUtilization();

    PhoenixUtil.registerSignals(false, temp, velocity, appliedVolts, statorCurrent, supplyCurrent);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
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
  public void runDutyCycle(double percent) {
    hopper.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void stop() {
    hopper.setControl(new NeutralOut());
  }
}

package frc.robot.subsystems.loader;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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

public class LoaderIOTalonFX implements LoaderIO {

  private final TalonFX leftLoader;
  private final TalonFX rightLoader;
  private final TalonFXConfiguration config;

  private final StatusSignal<Temperature> leftTemp;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftStatorCurrent;
  private final StatusSignal<Current> leftSupplyCurrent;

  private final StatusSignal<Temperature> rightTemp;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightStatorCurrent;
  private final StatusSignal<Current> rightSupplyCurrent;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(false);

  public LoaderIOTalonFX() {
    leftLoader = new TalonFX(Constants.Loader.LEFT_MOTOR_ID, Constants.Loader.CANBUS);
    rightLoader = new TalonFX(Constants.Loader.RIGHT_MOTOR_ID, Constants.Loader.CANBUS);
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Loader.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Loader.SUPPLY_LIMIT;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Loader.GEAR_RATIO;

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            leftLoader
                .getConfigurator()
                .apply(
                    config.withMotorOutput(
                        new MotorOutputConfigs()
                            .withInverted(Constants.Loader.LEFT_INVERTED)
                            .withNeutralMode(NeutralModeValue.Brake))));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            rightLoader
                .getConfigurator()
                .apply(
                    config.withMotorOutput(
                        new MotorOutputConfigs()
                            .withInverted(Constants.Loader.RIGHT_INVERTED)
                            .withNeutralMode(NeutralModeValue.Brake))));

    leftTemp = leftLoader.getDeviceTemp();
    leftVelocity = leftLoader.getVelocity();
    leftAppliedVolts = leftLoader.getMotorVoltage();
    leftStatorCurrent = leftLoader.getStatorCurrent();
    leftSupplyCurrent = leftLoader.getSupplyCurrent();

    rightTemp = rightLoader.getDeviceTemp();
    rightVelocity = rightLoader.getVelocity();
    rightAppliedVolts = rightLoader.getMotorVoltage();
    rightStatorCurrent = rightLoader.getStatorCurrent();
    rightSupplyCurrent = rightLoader.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / Constants.loopTime,
        leftTemp,
        leftVelocity,
        leftAppliedVolts,
        leftStatorCurrent,
        leftSupplyCurrent,
        rightTemp,
        rightVelocity,
        rightAppliedVolts,
        rightStatorCurrent,
        rightSupplyCurrent);

    leftLoader.optimizeBusUtilization();
    rightLoader.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        false,
        leftTemp,
        leftVelocity,
        leftAppliedVolts,
        leftStatorCurrent,
        leftSupplyCurrent,
        rightTemp,
        rightVelocity,
        rightAppliedVolts,
        rightStatorCurrent,
        rightSupplyCurrent);
  }

  @Override
  public void updateInputs(LoaderIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftTemp,
        leftVelocity,
        leftAppliedVolts,
        leftStatorCurrent,
        leftSupplyCurrent,
        rightTemp,
        rightVelocity,
        rightAppliedVolts,
        rightStatorCurrent,
        rightSupplyCurrent);

    inputs.leftConnected =
        BaseStatusSignal.isAllGood(
            leftTemp, leftVelocity, leftAppliedVolts, leftStatorCurrent, leftSupplyCurrent);
    inputs.leftTempCelsius = leftTemp.getValueAsDouble();
    inputs.leftVelocityRadsPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftStatorCurrentAmps = leftStatorCurrent.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();

    inputs.rightConnected =
        BaseStatusSignal.isAllGood(
            rightTemp, rightVelocity, rightAppliedVolts, rightStatorCurrent, rightSupplyCurrent);
    inputs.rightTempCelsius = rightTemp.getValueAsDouble();
    inputs.rightVelocityRadsPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightStatorCurrentAmps = rightStatorCurrent.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
  }

  @Override
  public void runVoltage(double volts) {
    leftLoader.setControl(voltageOut.withOutput(volts));
    rightLoader.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    leftLoader.setControl(new NeutralOut());
    rightLoader.setControl(new NeutralOut());
  }
}

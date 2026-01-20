package frc.robot.subsystems.IntakeLoader;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class LoaderIOTalonFX implements LoaderIO {

  private final TalonFX loader;
  private final TalonFXConfiguration config;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0).withEnableFOC(true);

  public LoaderIOTalonFX() {
    loader = new TalonFX(Constants.Loader.MOTOR_ID);
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Loader.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Loader.SUPPLY_LIMIT;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Loader.GEAR_RATIO;

    config.MotorOutput.Inverted = Constants.Loader.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    loader.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(LoaderIOInputs inputs) {
    inputs.connected = loader.isConnected();
    inputs.tempCelsius = loader.getDeviceTemp().getValueAsDouble();
    inputs.velocityRadsPerSec = Units.rotationsToRadians(loader.getVelocity().getValueAsDouble());
    inputs.appliedVolts = loader.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentAmps = loader.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = loader.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runDutyCycle(double percent) {
    loader.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void stop() {
    loader.setControl(new NeutralOut());
  }
}

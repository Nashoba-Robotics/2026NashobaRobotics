package frc.robot.subsystems.loader;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class LoaderIOTalonFX implements LoaderIO {
    
  private final TalonFX loader;
  private final TalonFXConfiguration config;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);

  public LoaderIOTalonFX() {
    loader = new TalonFX(Constants.Loader.MOTOR_ID);
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Loader.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Loader.SUPPLY_LIMIT;

    config.MotorOutput.Inverted = Constants.Loader.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0 = Constants.Loader.PID;

    loader.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(LoaderIOInputs inputs) {
    inputs.connected = loader.isConnected();
    inputs.tempCelsius = loader.getDeviceTemp().getValueAsDouble();
    inputs.appliedVolts = loader.getMotorVoltage().getValueAsDouble();
    inputs.velocityRadsPerSec = Units.rotationsToRadians(loader.getVelocity().getValueAsDouble());
    inputs.statorCurrentAmps = loader.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = loader.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runDutyCycle(double percent) {
    loader.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void runVelocity(double velocityRadsPerSec) {
    loader.setControl(velocityDutyCycle.withVelocity(Units.radiansToRotations(velocityRadsPerSec)));
  }

  @Override
  public void stop() {
    loader.setControl(new NeutralOut());
  }

  @Override
  public void setPID(double kP, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    loader.getConfigurator().apply(config);
  }
}

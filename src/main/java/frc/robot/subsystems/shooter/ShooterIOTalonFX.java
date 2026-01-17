package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooter;
  private final TalonFXConfiguration config;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);

  public ShooterIOTalonFX() {
    shooter = new TalonFX(Constants.Loader.MOTOR_ID);
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Shooter.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.SUPPLY_LIMIT;

    config.MotorOutput.Inverted = Constants.Shooter.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kP = Constants.Shooter.kP;
    config.Slot0.kD = Constants.Shooter.kD;
    config.Slot0.kS = Constants.Shooter.kS;
    config.Slot0.kG = Constants.Shooter.kG;
    config.Slot0.kV = Constants.Shooter.kV;
    config.Slot0.kA = Constants.Shooter.kA;

    shooter.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.connected = shooter.isConnected();
    inputs.tempCelsius = shooter.getDeviceTemp().getValueAsDouble();
    inputs.velocityRadPerSec = Units.rotationsToRadians(shooter.getVelocity().getValueAsDouble());
    inputs.velocitySetpointRadPerSec =
        Units.rotationsToRadians(shooter.getClosedLoopOutput().getValueAsDouble());
    inputs.appliedVolts = shooter.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentAmps = shooter.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = shooter.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runDutyCycle(double percent) {
    shooter.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void runVelocity(double velocityRadPerSec) {
    shooter.setControl(velocityDutyCycle.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void stop() {
    shooter.setControl(new NeutralOut());
  }

  @Override
  public void setPID(double kP, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    shooter.getConfigurator().apply(config);
  }

  @Override
  public void setFeedForward(double kS, double kG, double kV, double kA) {
    config.Slot0.kS = kS;
    config.Slot0.kG = kG;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    shooter.getConfigurator().apply(config);
  }
}

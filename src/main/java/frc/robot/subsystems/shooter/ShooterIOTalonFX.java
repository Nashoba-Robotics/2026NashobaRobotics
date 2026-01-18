package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooter;
  private final TalonFXConfiguration config;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withEnableFOC(true);

  public ShooterIOTalonFX() {
    shooter = new TalonFX(Constants.Loader.MOTOR_ID);
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Shooter.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.SUPPLY_LIMIT;

    config.MotorOutput.Inverted = Constants.Shooter.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.RotorToSensorRatio = Constants.Shooter.GEAR_RATIO;

    config.Slot0.kP = Constants.Shooter.kP;
    config.Slot0.kD = Constants.Shooter.kD;
    config.Slot0.kS = Constants.Shooter.kS;
    config.Slot0.kV = Constants.Shooter.kV;
    config.Slot0.kA = Constants.Shooter.kA;

    shooter.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.connected = shooter.isConnected();
    inputs.tempCelsius = shooter.getDeviceTemp().getValueAsDouble();
    inputs.velocityRadsPerSec = Units.rotationsToRadians(shooter.getVelocity().getValueAsDouble());
    inputs.velocitySetpointRadsPerSec =
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
  public void runVelocity(double velocityRadsPerSec) {
    shooter.setControl(velocityVoltage.withVelocity(Units.radiansToRotations(velocityRadsPerSec)));
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

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooter;
  private final TalonFXConfiguration config;

  private final StatusSignal<Temperature> temp;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Double> velocitySetpoint;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

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

    PhoenixUtil.tryUntilOk(5, () -> shooter.getConfigurator().apply(config, 0.25));

    temp = shooter.getDeviceTemp();
    velocity = shooter.getVelocity();
    velocitySetpoint = shooter.getClosedLoopReference();
    appliedVolts = shooter.getMotorVoltage();
    statorCurrent = shooter.getStatorCurrent();
    supplyCurrent = shooter.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / Constants.loopTime,
        temp,
        velocity,
        velocitySetpoint,
        appliedVolts,
        statorCurrent,
        supplyCurrent);
    shooter.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        false, temp, velocity, velocitySetpoint, appliedVolts, statorCurrent, supplyCurrent);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        temp, velocity, velocitySetpoint, appliedVolts, statorCurrent, supplyCurrent);

    inputs.connected =
        BaseStatusSignal.isAllGood(
            temp, velocity, velocitySetpoint, appliedVolts, statorCurrent, supplyCurrent);
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.velocitySetpointRadsPerSec =
        Units.rotationsToRadians(velocitySetpoint.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
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

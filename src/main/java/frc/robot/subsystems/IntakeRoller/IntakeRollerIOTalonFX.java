package frc.robot.subsystems.intakeRoller;

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

public class IntakeRollerIOTalonFX implements IntakeRollerIO {

  private final TalonFX roller;
  private final TalonFXConfiguration rollerConfig;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0).withEnableFOC(true);

  private final StatusSignal<Temperature> temp;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  public IntakeRollerIOTalonFX() {
    roller = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);

    rollerConfig = new TalonFXConfiguration();

    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.ROLLER_STATOR_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.ROLLER_STATOR_LIMIT;

    rollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rollerConfig.Feedback.SensorToMechanismRatio =
        Constants.Intake.ROLLER_GEAR_RATIO;

    rollerConfig.MotorOutput.Inverted = Constants.Intake.ROLLER_INVERTED;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // roller.getConfigurator().apply(rollerConfig);
    PhoenixUtil.tryUntilOk(5, () -> roller.getConfigurator().apply(rollerConfig));

    temp = roller.getDeviceTemp();
    velocity = roller.getVelocity();
    appliedVolts = roller.getMotorVoltage();
    statorCurrent = roller.getStatorCurrent();
    supplyCurrent = roller.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / Constants.loopTime, temp, velocity, appliedVolts, statorCurrent, supplyCurrent);

    roller.optimizeBusUtilization();

    PhoenixUtil.registerSignals(false, temp, velocity, appliedVolts, statorCurrent, supplyCurrent);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
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
    roller.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void stop() {
    roller.setControl(new NeutralOut());
  }
}

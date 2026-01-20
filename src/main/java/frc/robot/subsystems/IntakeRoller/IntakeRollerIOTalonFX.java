package frc.robot.subsystems.IntakeRoller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {

  private final TalonFX roller;
  private final TalonFXConfiguration rollerConfig;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0).withEnableFOC(true);

  public IntakeRollerIOTalonFX() {
    roller = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);

    rollerConfig = new TalonFXConfiguration();

    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.ROLLER_STATOR_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.ROLLER_STATOR_LIMIT;

    rollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rollerConfig.Feedback.SensorToMechanismRatio =
        Constants.Intake.ROLLER_SENSOR_TO_MECHANISM_GEAR_RATIO;

    rollerConfig.MotorOutput.Inverted = Constants.Intake.ROLLER_INVERTED;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    roller.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.connected = roller.isConnected();
    inputs.tempCelsius = roller.getDeviceTemp().getValueAsDouble();
    inputs.velocityRadsPerSec = Units.rotationsToRadians(roller.getVelocity().getValueAsDouble());
    inputs.appliedVolts = roller.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentAmps = roller.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = roller.getSupplyCurrent().getValueAsDouble();
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

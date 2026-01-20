package frc.robot.subsystems.Roller;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class RollerIOTalonFX implements RollerIO {

  private final TalonFX roller;
  private final TalonFX deploy;
  private final TalonFXConfiguration deployConfig;
  private final TalonFXConfiguration rollerConfig;

  private final CANcoder encoder;
  private final CANcoderConfiguration encoderConfig;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0).withEnableFOC(true);

  public RollerIOTalonFX() {
    roller = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);
    deploy = new TalonFX(Constants.Intake.DEPLOY_MOTOR_ID);

    rollerConfig = new TalonFXConfiguration();
    deployConfig = new TalonFXConfiguration();

    encoder = new CANcoder(Constants.Intake.DEPLOY_ENCODER_ID);
    encoderConfig = new CANcoderConfiguration();

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

    deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    deployConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.DEPLOY_STATOR_LIMIT;
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.DEPLOY_SUPPLY_LIMIT;

    deployConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    deployConfig.Feedback.FeedbackRemoteSensorID = Constants.Intake.DEPLOY_ENCODER_ID;
    deployConfig.Feedback.RotorToSensorRatio =
        Constants.Intake.DEPLOY_ROTOR_TO_MECHANISM_GEAR_RATIO;
    deployConfig.Feedback.SensorToMechanismRatio =
        Constants.Intake.DEPLOY_SENSOR_TO_MECHANISM_GEAR_RATIO;

    deployConfig.MotorOutput.Inverted = Constants.Intake.DEPLOY_INVERTED;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    deployConfig.Slot0.kP = Constants.Intake.kP;
    deployConfig.Slot0.kD = Constants.Intake.kD;
    deployConfig.Slot0.kS = Constants.Intake.kS;
    deployConfig.Slot0.kV = Constants.Intake.kV;
    deployConfig.Slot0.kA = Constants.Intake.kA;

    deploy.getConfigurator().apply(deployConfig);

    encoderConfig.MagnetSensor.SensorDirection = Constants.Intake.ENCODER_DIRECTION;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Intake.ENCODER_OFFSET;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        Constants.Intake.ENCODER_DISCONTINUITY_POINT;

    encoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerConnected = roller.isConnected();
    inputs.rollerTempCelsius = roller.getDeviceTemp().getValueAsDouble();
    inputs.rollerVelocityRadsPerSec =
        Units.rotationsToRadians(roller.getVelocity().getValueAsDouble());
    inputs.rollerAppliedVolts = roller.getMotorVoltage().getValueAsDouble();
    inputs.rollerStatorCurrentAmps = roller.getStatorCurrent().getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = roller.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runRollerDutyCycle(double percent) {
    roller.setControl(dutyCycle.withOutput(percent));
  }

  @Override
  public void rollerStop() {
    roller.setControl(new NeutralOut());
  }
}

package frc.robot.subsystems.intakeroller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {

  private final TalonFX rollerLeader;
  private final TalonFX rollerFollower;
  private final TalonFXConfiguration rollerConfig;

  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Double> velocitySetpoint;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Current> leaderSupplyCurrent;

  private final StatusSignal<Temperature> followerTemp;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(false);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

  public IntakeRollerIOTalonFX() {
    rollerLeader = new TalonFX(Constants.Intake.ROLLER_LEADER_MOTOR_ID, Constants.Intake.CANBUS);
    rollerFollower =
        new TalonFX(Constants.Intake.ROLLER_FOLLOWER_MOTOR_ID, Constants.Intake.CANBUS);
    rollerFollower.setControl(
        new Follower(Constants.Intake.ROLLER_LEADER_MOTOR_ID, MotorAlignmentValue.Opposed));

    rollerConfig = new TalonFXConfiguration();

    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.ROLLER_STATOR_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.ROLLER_STATOR_LIMIT;

    rollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rollerConfig.Feedback.SensorToMechanismRatio = Constants.Intake.ROLLER_GEAR_RATIO;

    rollerConfig.Slot0.kP = Constants.Intake.kP.get();
    rollerConfig.Slot0.kD = Constants.Intake.kD.get();
    rollerConfig.Slot0.kS = Constants.Intake.kS.get();
    rollerConfig.Slot0.kV = Constants.Intake.kV.get();
    rollerConfig.Slot0.kA = Constants.Intake.kA.get();

    rollerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    rollerConfig.MotorOutput.Inverted = Constants.Intake.ROLLER_INVERTED;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    PhoenixUtil.tryUntilOk(5, () -> rollerLeader.getConfigurator().apply(rollerConfig));
    PhoenixUtil.tryUntilOk(5, () -> rollerFollower.getConfigurator().apply(rollerConfig));
    // PhoenixUtil.tryUntilOk(
    //     5,
    //     () ->
    //         rollerFollower
    //             .getConfigurator()
    //             .apply(
    //                 rollerConfig.withMotorOutput(
    //                     new MotorOutputConfigs()
    //                         .withInverted(InvertedValue.CounterClockwise_Positive))));

    leaderTemp = rollerLeader.getDeviceTemp();
    leaderVelocity = rollerLeader.getVelocity();
    velocitySetpoint = rollerLeader.getClosedLoopReference();
    leaderAppliedVolts = rollerLeader.getMotorVoltage();
    leaderStatorCurrent = rollerLeader.getStatorCurrent();
    leaderSupplyCurrent = rollerLeader.getSupplyCurrent();

    followerTemp = rollerFollower.getDeviceTemp();
    followerVelocity = rollerFollower.getVelocity();
    followerAppliedVolts = rollerFollower.getMotorVoltage();
    followerStatorCurrent = rollerFollower.getStatorCurrent();
    followerSupplyCurrent = rollerFollower.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / Constants.loopTime,
        leaderTemp,
        leaderVelocity,
        velocitySetpoint,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        followerTemp,
        followerVelocity,
        followerAppliedVolts,
        followerStatorCurrent,
        followerSupplyCurrent);

    rollerLeader.optimizeBusUtilization();
    rollerFollower.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        false,
        leaderTemp,
        leaderVelocity,
        velocitySetpoint,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        followerTemp,
        followerVelocity,
        followerAppliedVolts,
        followerStatorCurrent,
        followerSupplyCurrent);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderTemp,
        leaderVelocity,
        velocitySetpoint,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        followerTemp,
        followerVelocity,
        followerAppliedVolts,
        followerStatorCurrent,
        followerSupplyCurrent);

    inputs.leaderConnected =
        BaseStatusSignal.isAllGood(
            leaderTemp,
            leaderVelocity,
            velocitySetpoint,
            leaderAppliedVolts,
            leaderStatorCurrent,
            leaderSupplyCurrent);
    inputs.leaderTempCelsius = leaderTemp.getValueAsDouble();
    inputs.leaderVelocityRadsPerSec = Units.rotationsToRadians(leaderVelocity.getValueAsDouble());
    inputs.velocitySetpointRadsPerSec =
        Units.rotationsToRadians(velocitySetpoint.getValueAsDouble());
    inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leaderStatorCurrent.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();

    inputs.followerConnected =
        BaseStatusSignal.isAllGood(
            followerTemp,
            followerVelocity,
            followerAppliedVolts,
            followerStatorCurrent,
            followerSupplyCurrent);
    inputs.followerTempCelsius = followerTemp.getValueAsDouble();
    inputs.followerVelocityRadsPerSec =
        Units.rotationsToRadians(followerVelocity.getValueAsDouble());
    inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.followerStatorCurrentAmps = followerStatorCurrent.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
  }

  @Override
  public void runVoltage(double volts) {
    rollerLeader.setControl(voltageOut.withOutput(volts));
    // rollerFollower.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runVelocity(double velocityRadsPerSec) {
    rollerLeader.setControl(
        velocityTorqueCurrentFOC.withVelocity(Units.radiansToRotations(velocityRadsPerSec)));
  }

  @Override
  public void stop() {
    rollerLeader.setControl(new NeutralOut());
    // rollerFollower.setControl(new NeutralOut());
  }

  @Override
  public void setPID(double kP, double kD) {
    rollerConfig.Slot0.kP = kP;
    rollerConfig.Slot0.kD = kD;
    rollerLeader.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void setFeedForward(double kS, double kG, double kV, double kA) {
    rollerConfig.Slot0.kS = kS;
    rollerConfig.Slot0.kG = kG;
    rollerConfig.Slot0.kV = kV;
    rollerConfig.Slot0.kA = kA;
    rollerLeader.getConfigurator().apply(rollerConfig);
  }
}

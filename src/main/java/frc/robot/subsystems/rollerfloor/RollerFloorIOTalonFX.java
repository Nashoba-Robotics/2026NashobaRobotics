package frc.robot.subsystems.rollerfloor;

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

public class RollerFloorIOTalonFX implements RollerFloorIO {

  private final TalonFX leader, follower;
  private final TalonFXConfiguration config;

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

  public RollerFloorIOTalonFX() {
    leader = new TalonFX(Constants.RollerFloor.LEADER_MOTOR_ID, Constants.RollerFloor.CANBUS);
    follower = new TalonFX(Constants.RollerFloor.FOLLOWER_MOTOR_ID, Constants.RollerFloor.CANBUS);
    follower.setControl(
        new Follower(Constants.RollerFloor.LEADER_MOTOR_ID, MotorAlignmentValue.Aligned));

    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.RollerFloor.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.RollerFloor.SUPPLY_LIMIT;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.RollerFloor.GEAR_RATIO;

    config.Slot0.kP = Constants.RollerFloor.kP.get();
    config.Slot0.kD = Constants.RollerFloor.kD.get();
    config.Slot0.kS = Constants.RollerFloor.kS.get();
    config.Slot0.kV = Constants.RollerFloor.kV.get();
    config.Slot0.kA = Constants.RollerFloor.kA.get();

    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    config.MotorOutput.Inverted = Constants.RollerFloor.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config));

    leaderTemp = leader.getDeviceTemp();
    leaderVelocity = leader.getVelocity();
    velocitySetpoint = leader.getClosedLoopReference();
    leaderAppliedVolts = leader.getMotorVoltage();
    leaderStatorCurrent = leader.getStatorCurrent();
    leaderSupplyCurrent = leader.getSupplyCurrent();

    followerTemp = follower.getDeviceTemp();
    followerVelocity = follower.getVelocity();
    followerAppliedVolts = follower.getMotorVoltage();
    followerStatorCurrent = follower.getStatorCurrent();
    followerSupplyCurrent = follower.getSupplyCurrent();

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

    leader.optimizeBusUtilization();

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
  public void updateInputs(RollerFloorIOInputs inputs) {
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
    leader.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runVelocity(double velocityRadsPerSec) {
    leader.setControl(
        velocityTorqueCurrentFOC.withVelocity(Units.radiansToRotations(velocityRadsPerSec)));
  }

  @Override
  public void stop() {
    leader.setControl(new NeutralOut());
  }

  @Override
  public void setPID(double kP, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    leader.getConfigurator().apply(config);
  }

  @Override
  public void setFeedForward(double kS, double kG, double kV, double kA) {
    config.Slot0.kS = kS;
    config.Slot0.kG = kG;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    leader.getConfigurator().apply(config);
  }
}

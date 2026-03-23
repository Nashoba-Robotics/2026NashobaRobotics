package frc.robot.subsystems.shooter;

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

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooterLeader, shooterFollower1, shooterFollower2, shooterFollower3;

  private final TalonFXConfiguration config;

  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Double> leaderVelocitySetpoint;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Current> leaderSupplyCurrent;

  private final StatusSignal<Temperature> follower1Temp;
  private final StatusSignal<AngularVelocity> follower1Velocity;
  private final StatusSignal<Voltage> follower1AppliedVolts;
  private final StatusSignal<Current> follower1StatorCurrent;
  private final StatusSignal<Current> follower1SupplyCurrent;

  private final StatusSignal<Temperature> follower2Temp;
  private final StatusSignal<AngularVelocity> follower2Velocity;
  private final StatusSignal<Voltage> follower2AppliedVolts;
  private final StatusSignal<Current> follower2StatorCurrent;
  private final StatusSignal<Current> follower2SupplyCurrent;

  private final StatusSignal<Temperature> follower3Temp;
  private final StatusSignal<AngularVelocity> follower3Velocity;
  private final StatusSignal<Voltage> follower3AppliedVolts;
  private final StatusSignal<Current> follower3StatorCurrent;
  private final StatusSignal<Current> follower3SupplyCurrent;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

  public ShooterIOTalonFX() {
    shooterLeader = new TalonFX(Constants.Shooter.SHOOTER_LEADER_ID, Constants.Shooter.CANBUS);
    shooterFollower1 = new TalonFX(Constants.Shooter.SHOOTER_FOLLOWER1_ID, Constants.Shooter.CANBUS);
    shooterFollower2 = new TalonFX(Constants.Shooter.SHOOTER_FOLLOWER2_ID, Constants.Shooter.CANBUS);
    shooterFollower3 = new TalonFX(Constants.Shooter.SHOOTER_FOLLOWER3_ID, Constants.Shooter.CANBUS);

    shooterFollower1.setControl(new Follower(Constants.Shooter.SHOOTER_LEADER_ID, MotorAlignmentValue.Aligned));
    shooterFollower2.setControl(new Follower(Constants.Shooter.SHOOTER_LEADER_ID, MotorAlignmentValue.Aligned));
    shooterFollower3.setControl(new Follower(Constants.Shooter.SHOOTER_LEADER_ID, MotorAlignmentValue.Aligned));

    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Shooter.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.SUPPLY_LIMIT;

    config.MotorOutput.Inverted = Constants.Shooter.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.RotorToSensorRatio = Constants.Shooter.GEAR_RATIO;

    config.Slot0.kP = Constants.Shooter.kP.get();
    config.Slot0.kD = Constants.Shooter.kD.get();
    config.Slot0.kS = Constants.Shooter.kS.get();
    config.Slot0.kV = Constants.Shooter.kV.get();
    config.Slot0.kA = Constants.Shooter.kA.get();

    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    PhoenixUtil.tryUntilOk(5, () -> shooterLeader.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> shooterFollower1.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> shooterFollower2.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> shooterFollower3.getConfigurator().apply(config, 0.25));

    leaderTemp = shooterLeader.getDeviceTemp();
    leaderVelocity = shooterLeader.getVelocity();
    leaderVelocitySetpoint = shooterLeader.getClosedLoopReference();
    leaderAppliedVolts = shooterLeader.getMotorVoltage();
    leaderStatorCurrent = shooterLeader.getStatorCurrent();
    leaderSupplyCurrent = shooterLeader.getSupplyCurrent();

    follower1Temp = shooterFollower1.getDeviceTemp();
    follower1Velocity = shooterFollower1.getVelocity();
    follower1AppliedVolts = shooterFollower1.getMotorVoltage();
    follower1StatorCurrent = shooterFollower1.getStatorCurrent();
    follower1SupplyCurrent = shooterFollower1.getSupplyCurrent();

    follower2Temp = shooterFollower2.getDeviceTemp();
    follower2Velocity = shooterFollower2.getVelocity();
    follower2AppliedVolts = shooterFollower2.getMotorVoltage();
    follower2StatorCurrent = shooterFollower2.getStatorCurrent();
    follower2SupplyCurrent = shooterFollower2.getSupplyCurrent();

    follower3Temp = shooterFollower3.getDeviceTemp();
    follower3Velocity = shooterFollower3.getVelocity();
    follower3AppliedVolts = shooterFollower3.getMotorVoltage();
    follower3StatorCurrent = shooterFollower3.getStatorCurrent();
    follower3SupplyCurrent = shooterFollower3.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / Constants.loopTime,
        leaderTemp,
        leaderVelocity,
        leaderVelocitySetpoint,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        follower1Temp,
        follower1Velocity,
        follower1AppliedVolts,
        follower1StatorCurrent,
        follower1SupplyCurrent,
        follower2Temp,
        follower2Velocity,
        follower2AppliedVolts,
        follower2StatorCurrent,
        follower2SupplyCurrent,
        follower3Temp,
        follower3Velocity,
        follower3AppliedVolts,
        follower3StatorCurrent,
        follower3SupplyCurrent);
        

    shooterLeader.optimizeBusUtilization();
    shooterFollower1.optimizeBusUtilization();
    shooterFollower2.optimizeBusUtilization();
    shooterFollower3.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        false,
        leaderTemp,
        leaderVelocity,
        leaderVelocitySetpoint,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        follower1Temp,
        follower1Velocity,
        follower1AppliedVolts,
        follower1StatorCurrent,
        follower1SupplyCurrent,
        follower2Temp,
        follower2Velocity,
        follower2AppliedVolts,
        follower2StatorCurrent,
        follower2SupplyCurrent,
        follower3Temp,
        follower3Velocity,
        follower3AppliedVolts,
        follower3StatorCurrent,
        follower3SupplyCurrent);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderTemp,
        leaderVelocity,
        leaderVelocitySetpoint,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        follower1Temp,
        follower1Velocity,
        follower1AppliedVolts,
        follower1StatorCurrent,
        follower1SupplyCurrent,
        follower2Temp,
        follower2Velocity,
        follower2AppliedVolts,
        follower2StatorCurrent,
        follower2SupplyCurrent,
        follower3Temp,
        follower3Velocity,
        follower3AppliedVolts,
        follower3StatorCurrent,
        follower3SupplyCurrent);

    inputs.leaderConnected =
        BaseStatusSignal.isAllGood(
            leaderTemp,
            leaderVelocity,
            leaderVelocitySetpoint,
            leaderAppliedVolts,
            leaderStatorCurrent,
            leaderSupplyCurrent);
    inputs.leaderTempCelsius = leaderTemp.getValueAsDouble();
    inputs.leaderVelocityRadsPerSec = Units.rotationsToRadians(leaderVelocity.getValueAsDouble());
    inputs.leaderVelocitySetpointRadsPerSec =
        Units.rotationsToRadians(leaderVelocitySetpoint.getValueAsDouble());
    inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leaderStatorCurrent.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();

    inputs.follower1Connected =
        BaseStatusSignal.isAllGood(
            follower1Temp,
            follower1Velocity,
            follower1AppliedVolts,
            follower1StatorCurrent,
            follower1SupplyCurrent);
    inputs.follower1TempCelsius = follower1Temp.getValueAsDouble();
    inputs.follower1VelocityRadsPerSec =
        Units.rotationsToRadians(follower1Velocity.getValueAsDouble());
    inputs.follower1AppliedVolts = follower1AppliedVolts.getValueAsDouble();
    inputs.follower1StatorCurrentAmps = follower1StatorCurrent.getValueAsDouble();
    inputs.follower1SupplyCurrentAmps = follower1SupplyCurrent.getValueAsDouble();

    inputs.follower2Connected =
        BaseStatusSignal.isAllGood(
            follower2Temp,
            follower2Velocity,
            follower2AppliedVolts,
            follower2StatorCurrent,
            follower2SupplyCurrent);
    inputs.follower2TempCelsius = follower2Temp.getValueAsDouble();
    inputs.follower2VelocityRadsPerSec =
        Units.rotationsToRadians(follower2Velocity.getValueAsDouble());
    inputs.follower2AppliedVolts = follower2AppliedVolts.getValueAsDouble();
    inputs.follower2StatorCurrentAmps = follower2StatorCurrent.getValueAsDouble();
    inputs.follower2SupplyCurrentAmps = follower2SupplyCurrent.getValueAsDouble();

    inputs.follower3Connected =
        BaseStatusSignal.isAllGood(
            follower3Temp,
            follower3Velocity,
            follower3AppliedVolts,
            follower3StatorCurrent,
            follower3SupplyCurrent);
    inputs.follower3TempCelsius = follower3Temp.getValueAsDouble();
    inputs.follower3VelocityRadsPerSec =
        Units.rotationsToRadians(follower3Velocity.getValueAsDouble());
    inputs.follower3AppliedVolts = follower3AppliedVolts.getValueAsDouble();
    inputs.follower3StatorCurrentAmps = follower3StatorCurrent.getValueAsDouble();
    inputs.follower3SupplyCurrentAmps = follower3SupplyCurrent.getValueAsDouble();
  }

  @Override
  public void runVoltage(double volts) {
    shooterLeader.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runVelocity(double velocityRadsPerSec) {
    shooterLeader.setControl(
        velocityTorqueCurrentFOC.withVelocity(Units.radiansToRotations(velocityRadsPerSec)));
  }

  @Override
  public void stop() {
    shooterLeader.setControl(new NeutralOut());
  }

  @Override
  public void setPID(double kP, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    shooterLeader.getConfigurator().apply(config);
  }

  @Override
  public void setFeedForward(double kS, double kG, double kV, double kA) {
    config.Slot0.kS = kS;
    config.Slot0.kG = kG;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    shooterLeader.getConfigurator().apply(config);
  }
}

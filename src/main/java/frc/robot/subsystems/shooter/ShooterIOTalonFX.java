package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooterLeader;
  private final TalonFX shooterFollower;

  private final TalonFXConfiguration config;

  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Double> leaderVelocitySetpoint;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Current> leaderSupplyCurrent;

  private final StatusSignal<Temperature> followerTemp;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withEnableFOC(true);

  public ShooterIOTalonFX(boolean isLeftShooter, int leaderDeviceId, int followerDeviceId) {
    shooterLeader = new TalonFX(leaderDeviceId, Constants.Shooter.CANBUS);
    shooterFollower = new TalonFX(followerDeviceId, Constants.Shooter.CANBUS);
    shooterFollower.setControl(new Follower(leaderDeviceId, MotorAlignmentValue.Aligned));

    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Shooter.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.SUPPLY_LIMIT;

    config.MotorOutput.Inverted =
        isLeftShooter ? Constants.Shooter.LEFT_INVERTED : Constants.Shooter.RIGHT_INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.RotorToSensorRatio = Constants.Shooter.GEAR_RATIO;

    config.Slot0.kP =
        isLeftShooter ? Constants.Shooter.LEFT_kP.get() : Constants.Shooter.RIGHT_kP.get();
    config.Slot0.kD =
        isLeftShooter ? Constants.Shooter.LEFT_kD.get() : Constants.Shooter.RIGHT_kD.get();
    config.Slot0.kS =
        isLeftShooter ? Constants.Shooter.LEFT_kS.get() : Constants.Shooter.RIGHT_kS.get();
    config.Slot0.kV =
        isLeftShooter ? Constants.Shooter.LEFT_kV.get() : Constants.Shooter.RIGHT_kV.get();
    config.Slot0.kA =
        isLeftShooter ? Constants.Shooter.LEFT_kA.get() : Constants.Shooter.RIGHT_kA.get();

    PhoenixUtil.tryUntilOk(5, () -> shooterLeader.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> shooterFollower.getConfigurator().apply(config, 0.25));

    leaderTemp = shooterLeader.getDeviceTemp();
    leaderVelocity = shooterLeader.getVelocity();
    leaderVelocitySetpoint = shooterLeader.getClosedLoopReference();
    leaderAppliedVolts = shooterLeader.getMotorVoltage();
    leaderStatorCurrent = shooterLeader.getStatorCurrent();
    leaderSupplyCurrent = shooterLeader.getSupplyCurrent();

    followerTemp = shooterFollower.getDeviceTemp();
    followerVelocity = shooterFollower.getVelocity();
    followerAppliedVolts = shooterFollower.getMotorVoltage();
    followerStatorCurrent = shooterFollower.getStatorCurrent();
    followerSupplyCurrent = shooterFollower.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / Constants.loopTime,
        leaderTemp,
        leaderVelocity,
        leaderVelocitySetpoint,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / Constants.loopTime,
        followerTemp,
        followerVelocity,
        followerAppliedVolts,
        followerStatorCurrent,
        followerSupplyCurrent);

    shooterLeader.optimizeBusUtilization();
    shooterFollower.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        false,
        leaderTemp,
        leaderVelocity,
        leaderVelocitySetpoint,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent);
    PhoenixUtil.registerSignals(
        false,
        followerTemp,
        followerVelocity,
        followerAppliedVolts,
        followerStatorCurrent,
        followerSupplyCurrent);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderTemp,
        leaderVelocity,
        leaderVelocitySetpoint,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent);
    BaseStatusSignal.refreshAll(
        followerTemp,
        followerVelocity,
        followerAppliedVolts,
        followerStatorCurrent,
        followerSupplyCurrent);

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
  public void runVoltage(double percent) {
    shooterLeader.setControl(voltageOut.withOutput(percent));
  }

  @Override
  public void runVelocity(double velocityRadsPerSec) {
    shooterLeader.setControl(
        velocityVoltage.withVelocity(Units.radiansToRotations(velocityRadsPerSec)));
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

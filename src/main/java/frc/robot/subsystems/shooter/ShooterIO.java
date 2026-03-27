package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public boolean leaderConnected = false;
    public double leaderTempCelsius = 0.0;
    public double leaderVelocityRadsPerSec = 0.0;
    public double velocitySetpointRadsPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderStatorCurrentAmps = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;

    public boolean follower1Connected = false;
    public double follower1TempCelsius = 0.0;
    public double follower1VelocityRadsPerSec = 0.0;
    public double follower1AppliedVolts = 0.0;
    public double follower1StatorCurrentAmps = 0.0;
    public double follower1SupplyCurrentAmps = 0.0;

    public boolean follower2Connected = false;
    public double follower2TempCelsius = 0.0;
    public double follower2VelocityRadsPerSec = 0.0;
    public double follower2AppliedVolts = 0.0;
    public double follower2StatorCurrentAmps = 0.0;
    public double follower2SupplyCurrentAmps = 0.0;

    public boolean follower3Connected = false;
    public double follower3TempCelsius = 0.0;
    public double follower3VelocityRadsPerSec = 0.0;
    public double follower3AppliedVolts = 0.0;
    public double follower3StatorCurrentAmps = 0.0;
    public double follower3SupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void runVoltage(double volts) {}

  public default void runVelocity(double velocityRadsPerSec) {}

  public default void stop() {}

  public default void setPID(double kP, double kD) {}

  public default void setFeedForward(double kS, double kG, double kV, double kA) {}
}

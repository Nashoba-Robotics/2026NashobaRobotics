package frc.robot.subsystems.entryfloor;

import org.littletonrobotics.junction.AutoLog;

public interface EntryFloorIO {

  @AutoLog
  public static class EntryFloorIOInputs {
    public boolean leaderConnected = false;
    public double leaderTempCelsius = 0.0;
    public double leaderVelocityRadsPerSec = 0.0;
    public double velocitySetpointRadsPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderStatorCurrentAmps = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;

    public boolean followerConnected = false;
    public double followerTempCelsius = 0.0;
    public double followerVelocityRadsPerSec = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public double followerSupplyCurrentAmps = 0.0;

    public boolean entryConnected = false;
    public double entryTempCelsius = 0.0;
    public double entryVelocityRadsPerSec = 0.0;
    public double entryAppliedVolts = 0.0;
    public double entryStatorCurrentAmps = 0.0;
    public double entrySupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(EntryFloorIOInputs inputs) {}

  public default void runVoltage(double volts) {}

  public default void runVelocity(double velocityRadsPerSec) {}

  public default void stop() {}

  public default void setPID(double kP, double kD) {}

  public default void setFeedForward(double kS, double kG, double kV, double kA) {}
}

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public boolean leaderConnected = false;
    public double leaderTempCelsius = 0.0;
    public double leaderVelocityRadsPerSec = 0.0;
    public double leaderVelocitySetpointRadsPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderStatorCurrentAmps = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;

    public boolean followerConnected = false;
    public double followerTempCelsius = 0.0;
    public double followerVelocityRadsPerSec = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void runDutyCycle(double percent) {}

  public default void runVelocity(double velocityRadsPerSec) {}

  public default void stop() {}

  public default void setPID(double kP, double kD) {}

  public default void setFeedForward(double kS, double kG, double kV, double kA) {}
}

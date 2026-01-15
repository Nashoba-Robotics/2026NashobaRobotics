package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {
    public boolean connected = false;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    // temporary values

  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void runDutyCycle(double percent) {}

  public default void runVelocity(double velocityRadsPerSec) {}

  public default void setPID(double kP, double kD) {}

  public default void stop() {}
}

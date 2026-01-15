package frc.robot.subsystems.loader;

import org.littletonrobotics.junction.AutoLog;

public interface LoaderIO {

  @AutoLog
  public static class LoaderIOInputs {
    public boolean connected = false;
    public double tempCelsius = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    // temporary values

  }

  public default void updateInputs(LoaderIOInputs inputs) {}

  public default void runDutyCycle(double percent) {}

  public default void runVelocity(double velocityRadsPerSec) {}

  public default void stop() {}

  public default void setPID(double kP, double kD) {}
}

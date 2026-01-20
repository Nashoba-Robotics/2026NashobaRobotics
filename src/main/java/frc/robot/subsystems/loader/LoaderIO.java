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
  }

  public default void updateInputs(LoaderIOInputs inputs) {}

  public default void runDutyCycle(double percent) {}

  public default void stop() {}
}

package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

  @AutoLog
  public static class SpindexerIOInputs {
    public boolean connected = false;
    public double tempCelsius = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void runVoltage(double volts) {}

  public default void stop() {}
}

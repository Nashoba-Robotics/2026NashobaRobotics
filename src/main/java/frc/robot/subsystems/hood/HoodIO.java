package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public boolean connected = false;
    public double tempCelsius = 0.0;
    public double absolutePositionRad = 0.0;
    public double rotorPositionRad = 0.0;
    public double positionSetpointRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void runDutyCycle(double percent) {}

  public default void runPosition(double positionRad) {}

  public default void stop() {}

  public default void setPID(double kP, double kD) {}

  public default void setFeedForward(double kS, double kG, double kV, double kA) {}
}

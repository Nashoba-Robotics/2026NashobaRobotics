package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public boolean connected = false;
    public double tempCelsius = 0.0;
    public double absolutePositionRads = 0.0;
    public double rotorPositionRads = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void runDutyCycle(double percent) {}

  public default void runPosition(double positionRads) {}

  public default void stop() {}

  public default void setPID(double kP, double kD) {}
}

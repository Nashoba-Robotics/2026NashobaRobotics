package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double tempCelsius = 0.0;
    public double velocityRadPerSec = 0.0;
    public double voltageVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    // temporary values
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPercentSpeed(double percent) {}
}

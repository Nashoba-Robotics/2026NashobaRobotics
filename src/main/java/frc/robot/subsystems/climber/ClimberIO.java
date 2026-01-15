package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIoInputs {
    public boolean connected = false;
    public double tempCelsius = 0.0;
    public double velocityRadPerSec = 0.0;
    public double voltageVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    // temporary values
  }

  public default void updateInputs(ClimberIoInputs inputs) {}
}

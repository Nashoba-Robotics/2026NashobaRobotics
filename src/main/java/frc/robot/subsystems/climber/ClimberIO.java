package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public boolean motorConnected = false;
    public boolean encoderConnected = false;
    public double tempCelsius = 0.0;
    public double absolutePositionRads = 0.0;
    public double rotorPositionRads = 0.0;
    public double positionSetpointRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void runVoltage(double voltage) {}

  public default void runPosition(double positionRads) {}

  public default void stop() {}

  public default void setPID(double kP, double kD) {}

  public default void setFeedForward(double kS, double kG, double kV, double kA) {}
}

package frc.robot.subsystems.intakedeploy;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeDeployIO {

  @AutoLog
  public static class IntakeDeployIOInputs {
    public boolean motorConnected = false;
    public double tempCelsius = 0.0;
    public double rotorPositionRads = 0.0;
    public double positionSetpointRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeDeployIOInputs inputs) {}

  public default void runVoltage(double percent) {}

  public default void runPosition(double positionRads) {}

  public default void stop() {}

  public default void setPID(double kP, double kD) {}

  public default void setFeedForward(double kS, double kG, double kV, double kA) {}
}

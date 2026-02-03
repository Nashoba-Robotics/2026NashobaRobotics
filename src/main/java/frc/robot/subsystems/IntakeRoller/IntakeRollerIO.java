package frc.robot.subsystems.intakeRoller;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {

  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean connected = false;
    public double tempCelsius = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  public default void runVoltage(double percent) {}

  public default void stop() {}
}

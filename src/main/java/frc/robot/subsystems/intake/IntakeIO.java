package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public boolean rollerConnected = false;
    public double rollerTempCelsius = 0.0;
    public double rollerVelocityRadsPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerStatorCurrentAmps = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;

    public boolean deployConnected = false;
    public double deployTempCelsius = 0.0;
    public double deployPositionRads = 0.0;
    public double deployVelocityRadsPerSec = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployStatorCurrentAmps = 0.0;
    public double deploySupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runDutyCycle(double percent) {}

  public default void runVelocity(double velocityRadsPerSec) {}

  public default void setPID(double kP, double kD) {}

  public default void stop() {}
}

package frc.robot.subsystems.Roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  @AutoLog
  public static class RollerIOInputs {
    public boolean rollerConnected = false;
    public double rollerTempCelsius = 0.0;
    public double rollerVelocityRadsPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerStatorCurrentAmps = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void runRollerDutyCycle(double percent) {}

  public default void rollerStop() {}
}

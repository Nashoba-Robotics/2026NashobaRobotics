package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public boolean rollerConnected = false;
    public double rollerTempCelsius = 0.0;
    public double rollerVelocityRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerStatorCurrentAmps = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;

    public boolean deployConnected = false;
    public double deployTempCelsius = 0.0;
    public double deployPositionRad = 0.0;
    public double deployPositionSetpointRad = 0.0;
    public double deployVelocityRadPerSec = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployStatorCurrentAmps = 0.0;
    public double deploySupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runRollerDutyCycle(double percent) {}

  public default void runDeployDutyCycle(double percent) {}

  public default void runDeployPosition(double positionRad) {}

  public default void rollerStop() {}

  public default void deployStop() {}

  public default void deploySetPID(double kP, double kD) {}

  public default void deploySetFeedForward(double kS, double kG, double kV, double kA) {}
}

package frc.robot.subsystems.IntakeDeploy;

import org.littletonrobotics.junction.AutoLog;

public interface DeployIO {

  @AutoLog
  public static class DeployIOInputs {
    public boolean deployConnected = false;
    public double deployTempCelsius = 0.0;
    public double deployPositionRads = 0.0;
    public double deployPositionSetpointRads = 0.0;
    public double deployVelocityRadsPerSec = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployStatorCurrentAmps = 0.0;
    public double deploySupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(DeployIOInputs inputs) {}

  public default void runDeployDutyCycle(double percent) {}

  public default void runDeployPosition(double positionRads) {}

  public default void deployStop() {}

  public default void deploySetPID(double kP, double kD) {}

  public default void deploySetFeedForward(double kS, double kG, double kV, double kA) {}
}

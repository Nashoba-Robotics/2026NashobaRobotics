package frc.robot.subsystems.loader;

import org.littletonrobotics.junction.AutoLog;

public interface LoaderIO {

  @AutoLog
  public static class LoaderIOInputs {
    public boolean leftConnected = false;
    public double leftTempCelsius = 0.0;
    public double leftVelocityRadsPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftStatorCurrentAmps = 0.0;
    public double leftSupplyCurrentAmps = 0.0;

    public boolean rightConnected = false;
    public double rightTempCelsius = 0.0;
    public double rightVelocityRadsPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightStatorCurrentAmps = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(LoaderIOInputs inputs) {}

  public default void runVoltage(double volts) {}

  public default void stop() {}
}

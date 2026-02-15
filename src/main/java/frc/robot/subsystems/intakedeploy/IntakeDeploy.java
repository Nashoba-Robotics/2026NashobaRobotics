package frc.robot.subsystems.intakedeploy;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeDeploy extends SubsystemBase {

  private final IntakeDeployIO io;
  private final IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Alert intakeDeployMotorDisconnectedAlert =
      new Alert("IntakeDeploy motor disconnected!", Alert.AlertType.kWarning);

  public IntakeDeploy(IntakeDeployIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeDeploy", inputs);

    intakeDeployMotorDisconnectedAlert.set(
        !motorConnectedDebouncer.calculate(inputs.motorConnected));

    if (Constants.Intake.kP.hasChanged(hashCode()) || Constants.Intake.kD.hasChanged(hashCode())) {
      io.setPID(Constants.Intake.kP.get(), Constants.Intake.kD.get());
    }
    if (Constants.Intake.kS.hasChanged(hashCode())
        || Constants.Intake.kV.hasChanged(hashCode())
        || Constants.Intake.kA.hasChanged(hashCode())) {
      io.setFeedForward(
          Constants.Intake.kS.get(), 0.0, Constants.Intake.kV.get(), Constants.Intake.kA.get());
    }
  }

  public boolean atSetpoint() {
    return Util.epsilonEquals(
        inputs.positionSetpointRads,
        inputs.rotorPositionRads,
        Units.degreesToRadians(Constants.Intake.POSITION_TOLERANCE.get()));
  }

  public Command runPositionCommand(double positionRads) {
    return run(() -> io.runPosition(positionRads)).until(this::atSetpoint);
  }

  public Command runTrackedPositionCommand(DoubleSupplier positionRads) {
    return run(() -> io.runPosition(positionRads.getAsDouble()));
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop());
  }
}

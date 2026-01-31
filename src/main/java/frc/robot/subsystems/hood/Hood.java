package frc.robot.subsystems.hood;

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

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Alert hoodMotorDisconnectedAlert =
      new Alert("Hood motor disconnected!", Alert.AlertType.kWarning);
  private final Alert hoodEncoderDisconnectedAlert =
      new Alert("Hood encoder disconnected!", Alert.AlertType.kWarning);

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    hoodMotorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    hoodEncoderDisconnectedAlert.set(!encoderConnectedDebouncer.calculate(inputs.encoderConnected));

    if (Constants.Hood.kP.hasChanged(hashCode()) || Constants.Hood.kD.hasChanged(hashCode())) {
      io.setPID(Constants.Hood.kP.get(), Constants.Hood.kD.get());
    }
    if (Constants.Hood.kS.hasChanged(hashCode())
        || Constants.Hood.kV.hasChanged(hashCode())
        || Constants.Hood.kA.hasChanged(hashCode())) {
      io.setFeedForward(
          Constants.Hood.kS.get(), 0.0, Constants.Hood.kV.get(), Constants.Hood.kA.get());
    }
  }

  public boolean atSetpoint() {
    return Util.epsilonEquals(
        inputs.positionSetpointRads,
        inputs.rotorPositionRads,
        Units.degreesToRadians(Constants.Hood.POSITION_TOLERANCE.get()));
  }

  public Command runPositionCommand(double positionRads) {
    return run(() -> io.runPosition(positionRads, 0.0))
        .until(
            () ->
                Util.epsilonEquals(
                    positionRads,
                    inputs.rotorPositionRads,
                    Units.degreesToRadians(Constants.Hood.POSITION_TOLERANCE.get())));
  }

  public Command runTrackedPositionCommand(
      DoubleSupplier positionRads, DoubleSupplier velocityRadsPerSec) {
    return run(() -> io.runPosition(positionRads.getAsDouble(), velocityRadsPerSec.getAsDouble()));
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop());
  }
}

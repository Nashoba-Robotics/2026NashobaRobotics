package frc.robot.subsystems.hood;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Tuning/Hood/kP", Constants.Hood.kP);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Tuning/Hood/kD", Constants.Hood.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Tuning/Hood/kS", Constants.Hood.kS);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Tuning/Hood/kV", Constants.Hood.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Tuning/Hood/kA", Constants.Hood.kA);

  private static final LoggedTunableNumber positionTolerance =
      new LoggedTunableNumber("Tuning/Hood/Tolerance", Constants.Hood.TOLERANCE);

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

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kD.get());
    }
    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
      io.setFeedForward(kS.get(), 0.0, kV.get(), kA.get());
    }
  }

  public boolean atSetpoint() {
    return Util.epsilonEquals(
        inputs.positionSetpointRads, inputs.rotorPositionRads, positionTolerance.get());
  }

  public Command runPositionCommand(double positionRads) {
    return run(() -> io.runPosition(positionRads, 0.0))
        .until(
            () ->
                Util.epsilonEquals(
                    positionRads, inputs.rotorPositionRads, positionTolerance.get()));
  }

  public Command runTrackedPositionCommand(
      DoubleSupplier positionRads, DoubleSupplier velocityRadsPerSec) {
    return run(() -> io.runPosition(positionRads.getAsDouble(), velocityRadsPerSec.getAsDouble()));
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop());
  }
}

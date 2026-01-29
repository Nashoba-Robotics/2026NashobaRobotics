package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Tuning/Climber/kP", Constants.Climber.kP);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Tuning/Climber/kD", Constants.Climber.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Tuning/Climber/kS", Constants.Climber.kS);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Tuning/Climber/kG", Constants.Climber.kG);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Tuning/Climber/kV", Constants.Climber.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Tuning/Climber/kA", Constants.Climber.kA);

  private static final LoggedTunableNumber positionTolerance =
      new LoggedTunableNumber("Tuning/Climber/ToleranceDeg", Constants.Climber.TOLERANCE);

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Alert climberMotorDisconnectedAlert =
      new Alert("Climber motor disconnected!", Alert.AlertType.kWarning);
  private final Alert climberEncoderDisconnectedAlert =
      new Alert("Climber encoder disconnected!", Alert.AlertType.kWarning);

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    climberMotorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    climberEncoderDisconnectedAlert.set(
        !encoderConnectedDebouncer.calculate(inputs.encoderConnected));

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kD.get());
    }
    if (kS.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      io.setFeedForward(kS.get(), kG.get(), kV.get(), kA.get());
    }
  }

  public boolean atSetpoint() {
    return Util.epsilonEquals(
        inputs.positionSetpointRads,
        inputs.rotorPositionRads,
        Units.degreesToRadians(positionTolerance.get()));
  }

  public Command runPositionCommand(double positionRads) {
    return run(() -> io.runPosition(positionRads))
        .until(
            () ->
                Util.epsilonEquals(
                    positionRads,
                    inputs.rotorPositionRads,
                    Units.degreesToRadians(positionTolerance.get())));
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop());
  }
}

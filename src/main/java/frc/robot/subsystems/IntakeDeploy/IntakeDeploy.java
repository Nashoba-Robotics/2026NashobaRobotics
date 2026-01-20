package frc.robot.subsystems.IntakeDeploy;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class IntakeDeploy extends SubsystemBase {

  private final IntakeDeployIO io;
  private final IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Tuning/Intake/kP", Constants.Intake.kP);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Tuning/Intake/kD", Constants.Intake.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Tuning/Intake/kS", Constants.Intake.kS);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Tuning/Intake/kV", Constants.Intake.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Tuning/Intake/kA", Constants.Intake.kA);

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert deployDisconnected =
      new Alert("IntakeDeploy motor disconnected!", Alert.AlertType.kWarning);

  public IntakeDeploy(IntakeDeployIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeDeploy", inputs);

    deployDisconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kD.get());
    }
    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
      io.setFeedForward(kS.get(), 0.0, kV.get(), kA.get());
    }
  }

  public Command runPositionCommand(double positionRads) {
    return run(() -> io.runPosition(positionRads))
        .until(
            () ->
                Util.epsilonEquals(
                    positionRads, inputs.rotorPositionRads, Constants.Intake.DEPLOY_TOLERANCE));
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop());
  }
}

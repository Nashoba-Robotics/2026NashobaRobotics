package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

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
  private final Alert rollerDisconnected =
      new Alert("IntakeRoller motor disconnected!", Alert.AlertType.kWarning);
  private final Alert deployDisconnected =
      new Alert("IntakeDeploy motor disconnected!", Alert.AlertType.kWarning);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    rollerDisconnected.set(!motorConnectedDebouncer.calculate(inputs.rollerConnected));
    deployDisconnected.set(!motorConnectedDebouncer.calculate(inputs.deployConnected));

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.deploySetPID(kP.get(), kD.get());
    }
    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
      io.deploySetFeedForward(kS.get(), 0.0, kV.get(), kA.get());
    }
  }

  public Command runRollerDutyCycleCommand(double percent){
        return run(() -> io.runRollerDutyCycle(percent));
    }

  public Command runDeployDutyCycleCommand(double percent){
        return run(() -> io.runDeployDutyCycle(percent));
    }

  public Command runDeployPositionCommand(double positionRads) {
    return run(() -> io.runDeployPosition(positionRads))
        .until(
            () ->
                Util.epsilonEquals(
                    positionRads, inputs.deployRotorPositionRads, Constants.Intake.DEPLOY_TOLERANCE));
  }

  public Command rollerStopCommand(){
    return runOnce(() -> io.rollerStop());
  }

  public Command deployStopCommand(){
    return runOnce(() -> io.deployStop());
  }
}

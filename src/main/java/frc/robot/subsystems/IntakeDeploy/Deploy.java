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

public class Deploy extends SubsystemBase {

  private final DeployIO io;
  private final DeployIOInputsAutoLogged inputs = new DeployIOInputsAutoLogged();

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

  public Deploy(DeployIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Deploy", inputs);

    deployDisconnected.set(!motorConnectedDebouncer.calculate(inputs.deployConnected));

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.deploySetPID(kP.get(), kD.get());
    }
    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
      io.deploySetFeedForward(kS.get(), 0.0, kV.get(), kA.get());
    }
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


  public Command deployStopCommand(){
    return runOnce(() -> io.deployStop());
  }
}

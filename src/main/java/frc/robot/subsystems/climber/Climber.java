package frc.robot.subsystems.climber;

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

  public Climber(ClimberIO climberIO) {
    io = climberIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

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

  public Command runPositionCommand(double positionRad) {
    return run(() -> io.runPosition(positionRad))
        .until(
            () ->
                Util.epsilonEquals(
                    positionRad, inputs.rotorPositionRad, Constants.Climber.TOLERANCE));
  }
}

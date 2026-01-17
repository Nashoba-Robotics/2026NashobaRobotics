package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
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
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Tuning/Intake/kG", Constants.Intake.kG);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Tuning/Intake/kV", Constants.Intake.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Tuning/Intake/kA", Constants.Intake.kA);

  public Intake(IntakeIO intakeIO) {
    io = intakeIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.deploySetPID(kP.get(), kD.get());
    }
    if (kS.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      io.deploySetFeedForward(kS.get(), kG.get(), kV.get(), kA.get());
    }
  }
}

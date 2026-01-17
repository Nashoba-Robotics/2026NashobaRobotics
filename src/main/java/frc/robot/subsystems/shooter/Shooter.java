package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Tuning/Shooter/kP", Constants.Shooter.kP);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Tuning/Shooter/kD", Constants.Shooter.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Tuning/Shooter/kS", Constants.Shooter.kS);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Tuning/Shooter/kG", Constants.Shooter.kG);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Tuning/Shooter/kV", Constants.Shooter.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Tuning/Shooter/kA", Constants.Shooter.kA);

  public Shooter(ShooterIO shooterIO) {
    io = shooterIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

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
}

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
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
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Tuning/Shooter/kV", Constants.Shooter.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Tuning/Shooter/kA", Constants.Shooter.kA);

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert disconnected =
      new Alert("Shooter motor disconnected!", Alert.AlertType.kWarning);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    disconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kD.get());
    }
    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
      io.setFeedForward(kS.get(), 0.0, kV.get(), kA.get());
    }
  }

  public Command runVelocityCommand(double velocityRadsPerSec) {
    return run(() -> io.runVelocity(velocityRadsPerSec));
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop());
  }
}

package frc.robot.subsystems.IntakeLoader;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Loader extends SubsystemBase {

  private final LoaderIO io;
  private final LoaderIOInputsAutoLogged inputs = new LoaderIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert loaderDisconnected =
      new Alert("Loader motor disconnected!", Alert.AlertType.kWarning);

  public Loader(LoaderIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Loader", inputs);
    loaderDisconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));
  }

  public Command runDutyCycleCommand(double percent) {
    return run(() -> io.runDutyCycle(percent));
  }
}

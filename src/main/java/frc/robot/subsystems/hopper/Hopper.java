package frc.robot.subsystems.hopper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private final Debouncer motorConectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Hopper motor disconnected!", Alert.AlertType.kWarning);

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    motorDisconnectedAlert.set(!motorConectedDebouncer.calculate(inputs.connected));
  }

  public Command runDutyCycleCommand(double percent) {
    return run(() -> io.runDutyCycle(percent));
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop());
  }
}

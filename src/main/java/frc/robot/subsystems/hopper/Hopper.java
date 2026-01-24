package frc.robot.subsystems.hopper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert hopperMotorDisconnectedAlert =
      new Alert("Hopper motor disconnected!", Alert.AlertType.kWarning);

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    hopperMotorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.connected));
  }

  public Command runDutyCycleCommand(DoubleSupplier percent) {
    return run(() -> io.runDutyCycle(percent.getAsDouble()));
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop());
  }
}

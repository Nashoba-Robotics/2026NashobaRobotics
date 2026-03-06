package frc.robot.subsystems.loader;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Loader extends SubsystemBase {

  private final LoaderIO io;
  private final LoaderIOInputsAutoLogged inputs = new LoaderIOInputsAutoLogged();

  private final Debouncer leftMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer rightMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Alert leftLoaderMotorDisconnectedAlert =
      new Alert("LeftLoader motor disconnected!", Alert.AlertType.kWarning);
  private final Alert rightLoaderMotorDisconnectedAlert =
      new Alert("RightLoader motor disconnected!", Alert.AlertType.kWarning);

  public Loader(LoaderIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Loader", inputs);

    leftLoaderMotorDisconnectedAlert.set(
        !leftMotorConnectedDebouncer.calculate(inputs.leftConnected));
    rightLoaderMotorDisconnectedAlert.set(
        !rightMotorConnectedDebouncer.calculate(inputs.rightConnected));
  }

  public Command runVoltageCommand(DoubleSupplier volts) {
    return run(() -> io.runVoltage(volts.getAsDouble())).finallyDo(() -> io.stop());
  }

  public void stop() {
    io.stop();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}

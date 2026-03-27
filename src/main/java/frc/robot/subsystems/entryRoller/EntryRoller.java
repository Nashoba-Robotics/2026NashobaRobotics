package frc.robot.subsystems.entryRoller;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class EntryRoller extends SubsystemBase {

  private final EntryRollerIO io;
  private final EntryRollerIOInputsAutoLogged inputs = new EntryRollerIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Alert entryRollerMotorDisconnectedAlert =
      new Alert("EntryRoller motor disconnected!", Alert.AlertType.kWarning);

  public EntryRoller(EntryRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EntryRoller", inputs);

    entryRollerMotorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.connected));

    if (Constants.EntryRoller.kP.hasChanged(hashCode())
        || Constants.EntryRoller.kD.hasChanged(hashCode())) {
      io.setPID(Constants.EntryRoller.kP.get(), Constants.EntryRoller.kD.get());
    }
    if (Constants.EntryRoller.kS.hasChanged(hashCode())
        || Constants.EntryRoller.kV.hasChanged(hashCode())
        || Constants.EntryRoller.kA.hasChanged(hashCode())) {
      io.setFeedForward(
          Constants.EntryRoller.kS.get(),
          0.0,
          Constants.EntryRoller.kV.get(),
          Constants.EntryRoller.kA.get());
    }
  }

  public boolean atSetpoint() {
    return Util.epsilonEquals(
        inputs.velocitySetpointRadsPerSec,
        inputs.velocityRadsPerSec,
        Constants.EntryRoller.VELOCITY_TOLERANCE.get());
  }

  public Command runVoltageCommand(DoubleSupplier volts) {
    return run(() -> io.runVoltage(volts.getAsDouble())).finallyDo(() -> io.stop());
  }

  public Command runVelocityCommand(double velocityRadsPerSec) {
    return run(() -> io.runVelocity(velocityRadsPerSec));
  }

  public void stop() {
    io.stop();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}

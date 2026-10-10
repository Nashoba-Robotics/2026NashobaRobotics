package frc.robot.subsystems.entryfloor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class EntryFloor extends SubsystemBase {
  private final EntryFloorIO io;
  private final EntryFloorIOInputsAutoLogged inputs = new EntryFloorIOInputsAutoLogged();

  private final Debouncer leaderMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer followerMotorConnectedDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer entryMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Alert leaderMotorDisconnectedAlert =
      new Alert("LeaderRollerFloor motor disconnected!", Alert.AlertType.kWarning);
  private final Alert followerMotorDisconnectedAlert =
      new Alert("LeaderRollerFloor motor disconnected!", Alert.AlertType.kWarning);
  private final Alert entryMotorDisconnectedAlert =
      new Alert("LeaderRollerFloor motor disconnected!", Alert.AlertType.kWarning);

  public EntryFloor(EntryFloorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EntryRoller", inputs);

    entryMotorDisconnectedAlert.set(!entryMotorConnectedDebouncer.calculate(inputs.entryConnected));
    leaderMotorDisconnectedAlert.set(
        !leaderMotorConnectedDebouncer.calculate(inputs.leaderConnected));
    followerMotorDisconnectedAlert.set(
        !followerMotorConnectedDebouncer.calculate(inputs.followerConnected));

    if (Constants.EntryFloor.kP.hasChanged(hashCode())
        || Constants.EntryFloor.kD.hasChanged(hashCode())) {
      io.setPID(Constants.EntryFloor.kP.get(), Constants.EntryFloor.kD.get());
    }
    if (Constants.EntryFloor.kS.hasChanged(hashCode())
        || Constants.EntryFloor.kV.hasChanged(hashCode())
        || Constants.EntryFloor.kA.hasChanged(hashCode())) {
      io.setFeedForward(
          Constants.EntryFloor.kS.get(),
          0.0,
          Constants.EntryFloor.kV.get(),
          Constants.EntryFloor.kA.get());
    }
  }

  public boolean atSetpoint() {
    return Util.epsilonEquals(
        inputs.velocitySetpointRadsPerSec,
        inputs.leaderVelocityRadsPerSec,
        Constants.EntryFloor.VELOCITY_TOLERANCE.get());
  }

  public Command runVoltageCommand(DoubleSupplier volts) {
    return run(() -> io.runVoltage(volts.getAsDouble())).finallyDo(() -> io.stop());
  }

  public Command runVelocityCommand(DoubleSupplier velocityRadsPerSec) {
    return run(() -> io.runVelocity(velocityRadsPerSec.getAsDouble()));
  }

  public void stop() {
    io.stop();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}

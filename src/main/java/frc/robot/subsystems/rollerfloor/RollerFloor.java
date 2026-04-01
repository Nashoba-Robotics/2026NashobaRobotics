package frc.robot.subsystems.rollerfloor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class RollerFloor extends SubsystemBase {

  private final RollerFloorIO io;
  private final RollerFloorIOInputsAutoLogged inputs = new RollerFloorIOInputsAutoLogged();

  private final Debouncer leaderMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer followerMotorConnectedDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);

  private final Alert leaderRollerFloorMotorDisconnectedAlert =
      new Alert("LeaderRollerFloor motor disconnected!", Alert.AlertType.kWarning);
  private final Alert followerRollerFloorMotorDisconnectedAlert =
      new Alert("FollowerRollerFloor motor disconnected!", Alert.AlertType.kWarning);

  public RollerFloor(RollerFloorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("RollerFloor", inputs);

    leaderRollerFloorMotorDisconnectedAlert.set(
        !leaderMotorConnectedDebouncer.calculate(inputs.leaderConnected));
    followerRollerFloorMotorDisconnectedAlert.set(
        !followerMotorConnectedDebouncer.calculate(inputs.followerConnected));

    if (Constants.RollerFloor.kP.hasChanged(hashCode())
        || Constants.RollerFloor.kD.hasChanged(hashCode())) {
      io.setPID(Constants.RollerFloor.kP.get(), Constants.RollerFloor.kD.get());
    }
    if (Constants.RollerFloor.kS.hasChanged(hashCode())
        || Constants.RollerFloor.kV.hasChanged(hashCode())
        || Constants.RollerFloor.kA.hasChanged(hashCode())) {
      io.setFeedForward(
          Constants.RollerFloor.kS.get(),
          0.0,
          Constants.RollerFloor.kV.get(),
          Constants.RollerFloor.kA.get());
    }
  }

  public boolean atSetpoint() {
    return Util.epsilonEquals(
        inputs.velocitySetpointRadsPerSec,
        inputs.leaderVelocityRadsPerSec,
        Constants.RollerFloor.VELOCITY_TOLERANCE.get());
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

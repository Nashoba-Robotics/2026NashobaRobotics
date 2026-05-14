package frc.robot.subsystems.intakeroller;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {

  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  private final Debouncer motorLeaderConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer motorFollowerConnectedDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);
  private final Alert rollerLeaderMotorDisconnectedAlert =
      new Alert("IntakeRoller Leader motor disconnected!", Alert.AlertType.kWarning);
  private final Alert rollerFollowerMotorDisconnectedAlert =
      new Alert("IntakeRoller Follower motor disconnected!", Alert.AlertType.kWarning);

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);

    rollerLeaderMotorDisconnectedAlert.set(
        !motorLeaderConnectedDebouncer.calculate(inputs.leaderConnected));
    rollerFollowerMotorDisconnectedAlert.set(
        !motorFollowerConnectedDebouncer.calculate(inputs.followerConnected));

    if (Constants.Intake.kP.hasChanged(hashCode()) || Constants.Intake.kD.hasChanged(hashCode())) {
      io.setPID(Constants.Intake.kP.get(), Constants.Intake.kD.get());
    }
    if (Constants.Intake.kS.hasChanged(hashCode())
        || Constants.Intake.kV.hasChanged(hashCode())
        || Constants.Intake.kA.hasChanged(hashCode())) {
      io.setFeedForward(
          Constants.Intake.kS.get(), 0.0, Constants.Intake.kV.get(), Constants.Intake.kA.get());
    }
  }

  public boolean atSetpoint() {
    return Util.epsilonEquals(
        inputs.velocitySetpointRadsPerSec,
        inputs.leaderVelocityRadsPerSec,
        Constants.Intake.VELOCITY_TOLERANCE.get());
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

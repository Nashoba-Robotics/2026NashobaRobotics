package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final Debouncer leaderMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer follower1MotorConnectedDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer follower2MotorConnectedDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer follower3MotorConnectedDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);

  private final Alert shooterLeaderDisconnectedAlert =
      new Alert("ShooterLeader motor disconnected!", Alert.AlertType.kWarning);
  ;
  private final Alert shooterFollower1DisconnectedAlert =
      new Alert("ShooterFollower1 motor disconnected!", Alert.AlertType.kWarning);
  private final Alert shooterFollower2DisconnectedAlert =
      new Alert("ShooterFollower2 motor disconnected!", Alert.AlertType.kWarning);
  private final Alert shooterFollower3DisconnectedAlert =
      new Alert("ShooterFollower3 motor disconnected!", Alert.AlertType.kWarning);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    shooterLeaderDisconnectedAlert.set(
        !leaderMotorConnectedDebouncer.calculate(inputs.leaderConnected));
    shooterFollower1DisconnectedAlert.set(
        !follower1MotorConnectedDebouncer.calculate(inputs.follower1Connected));
    shooterFollower2DisconnectedAlert.set(
        !follower2MotorConnectedDebouncer.calculate(inputs.follower2Connected));
    shooterFollower3DisconnectedAlert.set(
        !follower3MotorConnectedDebouncer.calculate(inputs.follower3Connected));

    if (Constants.Shooter.kP.hasChanged(hashCode())
        || Constants.Shooter.kD.hasChanged(hashCode())) {
      io.setPID(Constants.Shooter.kP.get(), Constants.Shooter.kD.get());
    }
    if (Constants.Shooter.kS.hasChanged(hashCode())
        || Constants.Shooter.kV.hasChanged(hashCode())
        || Constants.Shooter.kA.hasChanged(hashCode())) {
      io.setFeedForward(
          Constants.Shooter.kS.get(), 0.0, Constants.Shooter.kV.get(), Constants.Shooter.kA.get());
    }
  }

  public boolean atSetpoint() {
    return Util.epsilonEquals(
        inputs.velocitySetpointRadsPerSec,
        inputs.leaderVelocityRadsPerSec,
        Constants.Shooter.VELOCITY_TOLERANCE.get());
  }

  public boolean inTolerance(double tolerance) {
    return Util.epsilonEquals(
        inputs.velocitySetpointRadsPerSec, inputs.leaderVelocityRadsPerSec, tolerance);
  }

  public Command runVelocityCommand(double velocityRadsPerSec) {
    return run(() -> io.runVelocity(velocityRadsPerSec)).until(this::atSetpoint);
  }

  public Command runTrackedVelocityCommand(DoubleSupplier velocityRadsPerSec) {
    return run(() -> io.runVelocity(velocityRadsPerSec.getAsDouble()));
  }

  public void stop() {
    io.stop();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}

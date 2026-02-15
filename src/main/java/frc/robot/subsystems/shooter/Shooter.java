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

  private final boolean isLeftShooter;

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert shooterLeaderDisconnectedAlert;
  private final Alert shooterFollowerDisconnectedAlert;

  public Shooter(ShooterIO io, boolean isLeftShooter) {
    this.io = io;
    this.isLeftShooter = isLeftShooter;

    shooterLeaderDisconnectedAlert =
        new Alert(
            (isLeftShooter ? "Left" : "Right") + "ShooterLeader motor disconnected!",
            Alert.AlertType.kWarning);
    shooterFollowerDisconnectedAlert =
        new Alert(
            (isLeftShooter ? "Left" : "Right") + "ShooterFollower motor disconnected!",
            Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/" + (isLeftShooter ? "Left" : "Right"), inputs);

    shooterLeaderDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.leaderConnected));
    shooterFollowerDisconnectedAlert.set(
        !motorConnectedDebouncer.calculate(inputs.followerConnected));

    if (isLeftShooter) {
      if (Constants.Shooter.LEFT_kP.hasChanged(hashCode())
          || Constants.Shooter.LEFT_kD.hasChanged(hashCode())) {
        io.setPID(Constants.Shooter.LEFT_kP.get(), Constants.Shooter.LEFT_kD.get());
      }
      if (Constants.Shooter.LEFT_kS.hasChanged(hashCode())
          || Constants.Shooter.LEFT_kV.hasChanged(hashCode())
          || Constants.Shooter.LEFT_kA.hasChanged(hashCode())) {
        io.setFeedForward(
            Constants.Shooter.LEFT_kS.get(),
            0.0,
            Constants.Shooter.LEFT_kV.get(),
            Constants.Shooter.LEFT_kA.get());
      }
    } else {
      if (Constants.Shooter.RIGHT_kP.hasChanged(hashCode())
          || Constants.Shooter.RIGHT_kD.hasChanged(hashCode())) {
        io.setPID(Constants.Shooter.RIGHT_kP.get(), Constants.Shooter.RIGHT_kD.get());
      }
      if (Constants.Shooter.RIGHT_kS.hasChanged(hashCode())
          || Constants.Shooter.RIGHT_kV.hasChanged(hashCode())
          || Constants.Shooter.RIGHT_kA.hasChanged(hashCode())) {
        io.setFeedForward(
            Constants.Shooter.RIGHT_kS.get(),
            0.0,
            Constants.Shooter.RIGHT_kV.get(),
            Constants.Shooter.RIGHT_kA.get());
      }
    }
  }

  public boolean atSetpoint() {
    return Util.epsilonEquals(
        inputs.leaderVelocitySetpointRadsPerSec,
        inputs.leaderVelocityRadsPerSec,
        Constants.Shooter.VELOCITY_TOLERANCE.get());
  }

  public Command runVelocityCommand(double velocityRadsPerSec) {
    return run(() -> io.runVelocity(velocityRadsPerSec)).until(this::atSetpoint);
  }

  public Command runTrackedVelocityCommand(DoubleSupplier velocityRadsPerSec) {
    return run(() -> io.runVelocity(velocityRadsPerSec.getAsDouble()));
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop());
  }
}

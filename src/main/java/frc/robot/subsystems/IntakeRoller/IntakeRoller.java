package frc.robot.subsystems.IntakeRoller;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {

  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert intakeRollerMotorDisconnected =
      new Alert("IntakeRoller motor disconnected!", Alert.AlertType.kWarning);

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);

    intakeRollerMotorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));
  }

  public Command runDutyCycleCommand(double percent) {
    return run(() -> io.runDutyCycle(percent));
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop());
  }
}

package frc.robot.subsystems.Roller;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {

  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert rollerDisconnected =
      new Alert("IntakeRoller motor disconnected!", Alert.AlertType.kWarning);

  public Roller(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);

    rollerDisconnected.set(!motorConnectedDebouncer.calculate(inputs.rollerConnected));
  }
}

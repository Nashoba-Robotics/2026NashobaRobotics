package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Tuning/Hood/kP", Constants.Hood.kP);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Tuning/Hood/kD", Constants.Hood.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Tuning/Hood/kS", Constants.Hood.kS);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Tuning/Hood/kG", Constants.Hood.kG);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Tuning/Hood/kV", Constants.Hood.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Tuning/Hood/kA", Constants.Hood.kA);

  public Hood(HoodIO hoodIO) {
    io = hoodIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kD.get());
    }
    if (kS.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      io.setFeedForward(kS.get(), kG.get(), kV.get(), kA.get());
    }
  }
}

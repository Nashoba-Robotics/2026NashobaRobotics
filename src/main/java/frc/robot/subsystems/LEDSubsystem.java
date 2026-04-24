package frc.robot.subsystems;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle candle;
  // set to false to disable LEDs
  private final boolean periodicEnabled = true;
  private int LEDCount = 75;

  private EmptyAnimation disabled = new EmptyAnimation(0);
  private RainbowAnimation rainbow = new RainbowAnimation(0, LEDCount);
  private FireAnimation fire = new FireAnimation(0, LEDCount);
  private LarsonAnimation larson = new LarsonAnimation(0, LEDCount);
  private SolidColor color = new SolidColor(0, LEDCount);
  private StrobeAnimation strobe = new StrobeAnimation(0, LEDCount);

  public LEDSubsystem() {
    candle = new CANdle(Constants.Led.DEVICE_ID, Constants.Led.CANBUS);
  }

  public Command disableLEDsCommand() {
    return run(() -> candle.setControl(disabled));
  }

  public Command setRainbowCommand(double brightness) {
    disableLEDsCommand();
    return run(() -> candle.setControl(rainbow.withBrightness(brightness).withSlot(0)));
  }

  public Command setFireCommand() {
    disableLEDsCommand();
    return run(() -> candle.setControl(fire));
  }

  public Command setLarsonCommand(int r, int g, int b, int speed, int size) {
    disableLEDsCommand();
    return run(
        () ->
            candle.setControl(
                larson
                    .withColor(new RGBWColor(r, g, b))
                    .withSize(size)
                    .withBounceMode(LarsonBounceValue.Center)));
  }

  public Command setColor(int r, int g, int b) {
    disableLEDsCommand();
    return run(() -> candle.setControl(color.withColor(new RGBWColor(r, g, b))));
  }

  public Command setStrobeAnimation(int r, int g, int b, int frameRate) {
    disableLEDsCommand();
    return run(
        () ->
            candle.setControl(
                strobe.withColor(new RGBWColor(r, g, b)).withFrameRate(frameRate).withSlot(0)));
  }

  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in
    // teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  boolean isAllianceBlue() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        return false;
      }
      if (alliance.get() == Alliance.Blue) {
        return true;
      }
    } // Field error return blue
    return true;
  }

  @Override
  public void periodic() {

    setRainbowCommand(50);

    if (periodicEnabled == false) return;

    boolean isHubActive = isHubActive();
    Logger.recordOutput("Hubactive Flag", isHubActive);

    if (isHubActive == true) {
      boolean isAllianceBlue = isAllianceBlue();
      if (isAllianceBlue == true) {
        // On Blue Alliance setting color blue
        candle.setControl(color.withColor(new RGBWColor(0, 0, 255)));
      } else {
        // On Red Alliance setting color red
        candle.setControl(color.withColor(new RGBWColor(255, 0, 0)));
      }
    } else {
      // Inacttive hub we set color to white
      candle.setControl(color.withColor(new RGBWColor(255, 255, 255)));
    }
  }
}

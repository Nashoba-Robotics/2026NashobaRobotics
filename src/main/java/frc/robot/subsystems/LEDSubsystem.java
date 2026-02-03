package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle candle;

  private Superstructure superstructure;
  private CANBus canBus = new CANBus();

  private int LEDCount = 0;

  private EmptyAnimation disabled = new EmptyAnimation(0);
  private RainbowAnimation rainbow = new RainbowAnimation(0, LEDCount);
  private FireAnimation fire = new FireAnimation(0, LEDCount);
  private LarsonAnimation larson = new LarsonAnimation(0, LEDCount);
  private SolidColor color = new SolidColor(0, LEDCount);

  public LEDSubsystem() {
    candle = new CANdle(0, canBus);
  }

  public Command disableLEDsCommand() {
    return run(() -> candle.setControl(disabled));
  }

  public Command setRainbowCommand(double brightness) {
    disableLEDsCommand();
    return run(() -> candle.setControl(rainbow.withBrightness(brightness)));
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
}

package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.drive.Drive;

public class TestAuto extends AutoModeBase {
  public TestAuto(Drive drive, AutoFactory factory) {
    super(factory, "test auto");

    AutoTrajectory test1 = trajectory("Test1");
    AutoTrajectory test2 = trajectory("Test2");

    newRoutine(
        // test1.resetOdometry(),
        cmdWithAccuracy(drive, test1, Units.Seconds.of(4.0), Units.Centimeters.of(3.0)),
        cmdWithAccuracy(drive, test2, Units.Seconds.of(4.0), Units.Centimeters.of(3.0)));
  }
}

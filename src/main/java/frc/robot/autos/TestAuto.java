package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import frc.robot.subsystems.drive.Drive;

public class TestAuto extends AutoModeBase {
  public TestAuto(Drive drive, AutoFactory factory) {
    super(factory, "TestAuto");

    AutoTrajectory tunePath = trajectory("TUNEPATH");
    newRoutine(tunePath.resetOdometry(), cmdWithAccuracy(drive, tunePath));
  }
}

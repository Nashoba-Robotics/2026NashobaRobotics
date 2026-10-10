package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class MiddleAuto extends AutoModeBase {
  public MiddleAuto(Drive drive, Superstructure superstructure, AutoFactory factory) {
    super(factory, "MiddleAuto");

    AutoTrajectory middlePath = trajectory("Middle", false);
    newRoutine(
        middlePath.resetOdometry(), cmdWithAccuracy(drive, middlePath), superstructure.autoShoot());
  }
}

package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class TestAuto extends AutoModeBase {
  public TestAuto(Drive drive, Superstructure superstructure, AutoFactory factory) {
    super(factory, "test auto");

    AutoTrajectory right_T_NZ = trajectory("Right_T_NZ");
    AutoTrajectory right_Safe_NZ_T = trajectory("Right_Safe_NZ_T");
    AutoTrajectory right_2nd_T_NZ = trajectory("Right_2nd_T_NZ");

    newRoutine(
        right_T_NZ.resetOdometry(),
        cmdWithAccuracy(drive, right_T_NZ, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
        cmdWithAccuracy(drive, right_Safe_NZ_T),
        new ParallelCommandGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(new WaitCommand(AutoConstants.kDelayIntakeRetract)),
            superstructure.autoRetractIntake()),
        cmdWithAccuracy(drive, right_2nd_T_NZ, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
        cmdWithAccuracy(drive, right_Safe_NZ_T),
        new ParallelCommandGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(new WaitCommand(AutoConstants.kDelayIntakeRetract)),
            superstructure.autoRetractIntake()));
  }
}

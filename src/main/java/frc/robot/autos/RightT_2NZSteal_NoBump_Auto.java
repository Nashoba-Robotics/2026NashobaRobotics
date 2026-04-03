package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class RightT_2NZSteal_NoBump_Auto extends AutoModeBase {
  public RightT_2NZSteal_NoBump_Auto(
      Drive drive, Superstructure superstructure, AutoFactory factory) {
    super(factory, "Right Steal DoubleSweep NoBump");

    AutoTrajectory right_T_NZSteal_T = trajectory("Right_T_NZSteal_T");
    AutoTrajectory right_Safe_Trench = trajectory("Right_Safe_Trench");
    AutoTrajectory right_2nd_T_NZ_T = trajectory("Right_2nd_T_NZ_T");
    newRoutine(
        right_T_NZSteal_T.resetOdometry(),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, right_T_NZSteal_T, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
                new SequentialCommandGroup(new WaitCommand(0.65), superstructure.autoRunIntake()))
            .until(drive::isBeached)
            .handleInterrupt(() -> antiBeach(drive)),
        cmdWithAccuracy(drive, right_Safe_Trench),
        new ParallelCommandGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                superstructure.autoShakeIntake())),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, right_2nd_T_NZ_T, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
                new SequentialCommandGroup(new WaitCommand(1.25), superstructure.autoRunIntake()))
            .until(drive::isBeached)
            .handleInterrupt(() -> antiBeach(drive)),
        cmdWithAccuracy(drive, right_Safe_Trench),
        new ParallelCommandGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                superstructure.autoShakeIntake())));
  }
}

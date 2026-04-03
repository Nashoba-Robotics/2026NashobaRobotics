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

public class LeftT_2NZSteal_NoBump_Auto extends AutoModeBase {
  public LeftT_2NZSteal_NoBump_Auto(
      Drive drive, Superstructure superstructure, AutoFactory factory) {
    super(factory, "Left Steal DoubleSweep NoBump");

    AutoTrajectory left_T_NZSteal_T = trajectory("Left_T_NZSteal_T");
    AutoTrajectory left_Safe_Trench = trajectory("Left_Safe_Trench");
    AutoTrajectory left_2nd_T_NZ_T = trajectory("Left_2nd_T_NZ_T");
    newRoutine(
        left_T_NZSteal_T.resetOdometry(),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, left_T_NZSteal_T, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
                new SequentialCommandGroup(new WaitCommand(0.65), superstructure.autoRunIntake()))
            .until(drive::isBeached)
            .handleInterrupt(() -> antiBeach(drive)),
        cmdWithAccuracy(drive, left_Safe_Trench),
        new ParallelCommandGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                superstructure.autoShakeIntake())),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, left_2nd_T_NZ_T, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
                new SequentialCommandGroup(new WaitCommand(1.50), superstructure.autoRunIntake()))
            .until(drive::isBeached)
            .handleInterrupt(() -> antiBeach(drive)),
        cmdWithAccuracy(drive, left_Safe_Trench),
        new ParallelCommandGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                superstructure.autoShakeIntake())));
  }
}

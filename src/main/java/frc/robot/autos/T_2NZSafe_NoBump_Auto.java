package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class T_2NZSafe_NoBump_Auto extends AutoModeBase {
  public T_2NZSafe_NoBump_Auto(
      Drive drive, Superstructure superstructure, AutoFactory factory, boolean isLeft) {
    super(factory, (isLeft ? "Left " : "Right ") + "Safe DoubleSweep NoBump");

    AutoTrajectory T_NZSafe_T = trajectory("T_NZSafe_T", isLeft);
    AutoTrajectory Safe_Trench = trajectory("Safe_Trench", isLeft);
    AutoTrajectory second_T_NZ_T = trajectory("Second_T_NZ_T", isLeft);
    AutoTrajectory antiBeach_Safe = trajectory("AntiBeach_Trench", isLeft);

    newRoutine(
        T_NZSafe_T.resetOdometry(),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, T_NZSafe_T, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
                new SequentialCommandGroup(new WaitCommand(0.60), superstructure.autoRunIntake()))
            .until(drive::isBeached),
        antiBeach(drive, antiBeach_Safe),
        cmdWithAccuracy(drive, Safe_Trench),
        new ParallelDeadlineGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                superstructure.autoRetractIntake())),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, second_T_NZ_T, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
                new SequentialCommandGroup(new WaitCommand(1.25), superstructure.autoRunIntake()))
            .until(drive::isBeached),
        antiBeach(drive, antiBeach_Safe),
        cmdWithAccuracy(drive, Safe_Trench),
        new ParallelDeadlineGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                superstructure.autoRetractIntake())),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, second_T_NZ_T, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
                new SequentialCommandGroup(new WaitCommand(1.25), superstructure.autoRunIntake()))
            .until(drive::isBeached),
        antiBeach(drive, antiBeach_Safe));
  }
}

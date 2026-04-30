package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class T_2NZSteal_Bump_Auto extends AutoModeBase {
  public T_2NZSteal_Bump_Auto(
      Drive drive,
      Superstructure superstructure,
      AutoFactory factory,
      boolean isLeft,
      boolean isGreedy) {
    super(factory, (isLeft ? "Left " : "Right ") + "Steal DoubleSweep Bump");

    AutoTrajectory T_NZSteal_B = trajectory("T_NZSteal_B", isLeft);
    AutoTrajectory safe_Bump = trajectory("Safe_Bump", isLeft);
    AutoTrajectory second_T_NZ_B =
        (isGreedy
            ? trajectory("Greedy_Second_T_NZ_B", isLeft)
            : trajectory("Second_T_NZ_B", isLeft));
    AutoTrajectory end_T_NZ = trajectory("End_T_NZ", isLeft);
    AutoTrajectory antiBeach_Safe = trajectory("AntiBeach_Bump", isLeft);

    newRoutine(
        T_NZSteal_B.resetOdometry(),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive,
                    T_NZSteal_B,
                    AutoConstants.kBumpLinearEpsilon,
                    AutoConstants.kBumpAngleEpsilon),
                new SequentialCommandGroup(new WaitCommand(0.50), superstructure.autoRunIntake()))
            .until(drive::isBeached),
        antiBeach(drive, antiBeach_Safe),
        new ParallelDeadlineGroup(
            cmdWithAccuracy(
                drive,
                safe_Bump,
                AutoConstants.kBumpLinearEpsilon,
                AutoConstants.kBumpAngleEpsilon),
            superstructure.autoRunIntake()),
        new ParallelDeadlineGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                superstructure.autoRetractIntake())),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive,
                    second_T_NZ_B,
                    AutoConstants.kBumpLinearEpsilon,
                    AutoConstants.kBumpAngleEpsilon),
                superstructure.autoRunIntake())
            .until(drive::isBeached),
        antiBeach(drive, antiBeach_Safe),
        new ParallelDeadlineGroup(
            cmdWithAccuracy(
                drive,
                safe_Bump,
                AutoConstants.kBumpLinearEpsilon,
                AutoConstants.kBumpAngleEpsilon),
            superstructure.autoRunIntake()),
        new ParallelDeadlineGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                superstructure.retractIntake())),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive,
                    end_T_NZ,
                    AutoConstants.kBumpLinearEpsilon,
                    AutoConstants.kBumpAngleEpsilon),
                superstructure.autoRunIntake())
            .until(drive::isBeached),
        antiBeach(drive, antiBeach_Safe));
  }
}

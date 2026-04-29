package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class T_2NZSafe_Bump_Auto extends AutoModeBase {
  public T_2NZSafe_Bump_Auto(
      Drive drive, Superstructure superstructure, AutoFactory factory, boolean isLeft) {
    super(factory, (isLeft ? "Left " : "Right ") + "Safe DoubleSweep Bump");

    AutoTrajectory T_NZSafe_B = trajectory("T_NZSafe_B", isLeft);
    AutoTrajectory safe_Bump = trajectory("Safe_Bump", isLeft);
    AutoTrajectory second_T_NZ_B = trajectory("Second_T_NZ_B", isLeft);
    AutoTrajectory end_T_NZ = trajectory("End_T_NZ", isLeft);
    AutoTrajectory antiBeach_Safe = trajectory("AntiBeach_Bump", isLeft);

    newRoutine(
        T_NZSafe_B.resetOdometry(),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive,
                    T_NZSafe_B,
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
                superstructure.autoRetractIntake())),
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

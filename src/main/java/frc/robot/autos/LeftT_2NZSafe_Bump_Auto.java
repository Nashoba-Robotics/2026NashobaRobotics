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

public class LeftT_2NZSafe_Bump_Auto extends AutoModeBase {
  public LeftT_2NZSafe_Bump_Auto(Drive drive, Superstructure superstructure, AutoFactory factory) {
    super(factory, "Left Safe DoubleSweep Bump");

    AutoTrajectory left_T_NZSafe_B = trajectory("Left_T_NZSafe_B");
    AutoTrajectory left_Safe_Bump = trajectory("Left_Safe_Bump");
    AutoTrajectory left_2nd_T_NZ_B = trajectory("Left_2nd_T_NZ_B");
    newRoutine(
        left_T_NZSafe_B.resetOdometry(),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, left_T_NZSafe_B, Units.Seconds.of(20.0), Units.Centimeters.of(25.0)),
                new SequentialCommandGroup(new WaitCommand(0.60), superstructure.autoRunIntake()))
            .until(drive::isBeached)
            .handleInterrupt(() -> antiBeach(drive)),
        cmdWithAccuracy(drive, left_Safe_Bump),
        new ParallelCommandGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                superstructure.autoShakeIntake())),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, left_2nd_T_NZ_B, Units.Seconds.of(20.0), Units.Centimeters.of(25.0)),
                superstructure.autoRunIntake())
            .until(drive::isBeached)
            .handleInterrupt(() -> antiBeach(drive)),
        cmdWithAccuracy(drive, left_Safe_Bump),
        new ParallelCommandGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                superstructure.autoShakeIntake())),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, left_2nd_T_NZ_B, Units.Seconds.of(20.0), Units.Centimeters.of(25.0)),
                superstructure.autoRunIntake())
            .until(drive::isBeached)
            .handleInterrupt(() -> antiBeach(drive)));
  }
}

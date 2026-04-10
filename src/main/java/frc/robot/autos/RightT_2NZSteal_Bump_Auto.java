package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class RightT_2NZSteal_Bump_Auto extends AutoModeBase {
  public RightT_2NZSteal_Bump_Auto(
      Drive drive, Superstructure superstructure, AutoFactory factory) {
    super(factory, "Right Steal DoubleSweep Bump");

    AutoTrajectory right_T_NZSteal_B = trajectory("Right_T_NZSteal_B");
    AutoTrajectory right_Safe_Bump = trajectory("Right_Safe_Bump");
    AutoTrajectory right_2nd_T_NZ_B = trajectory("Right_2nd_T_NZ_B");
    newRoutine(
        right_T_NZSteal_B.resetOdometry(),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, right_T_NZSteal_B, Units.Seconds.of(20.0), Units.Centimeters.of(25.0)),
                new SequentialCommandGroup(new WaitCommand(0.60), superstructure.autoRunIntake()))
            .until(drive::isBeached)
            .handleInterrupt(() -> antiBeach(drive)),
        cmdWithAccuracy(drive, right_Safe_Bump),
        new ParallelDeadlineGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                superstructure.autoRetractIntake())),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, right_2nd_T_NZ_B, Units.Seconds.of(20.0), Units.Centimeters.of(25.0)),
                superstructure.autoRunIntake())
            .until(drive::isBeached)
            .handleInterrupt(() -> antiBeach(drive)),
        cmdWithAccuracy(drive, right_Safe_Bump),
        new ParallelDeadlineGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                superstructure.retractIntake())),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, right_2nd_T_NZ_B, Units.Seconds.of(20.0), Units.Centimeters.of(25.0)),
                superstructure.autoRunIntake())
            .until(drive::isBeached)
            .handleInterrupt(() -> antiBeach(drive)));
  }
}

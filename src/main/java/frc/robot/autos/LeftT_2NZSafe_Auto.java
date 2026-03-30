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

public class LeftT_2NZSafe_Auto extends AutoModeBase {
  public LeftT_2NZSafe_Auto(Drive drive, Superstructure superstructure, AutoFactory factory) {
    super(factory, "LeftT_2NZSafe_Auto");

    AutoTrajectory left_T_NZSafe = trajectory("Left_T_NZSafe");
    AutoTrajectory left_Safe_NZ_T = trajectory("Left_Safe_NZ_T");
    AutoTrajectory left_2nd_T_NZ = trajectory("Left_2nd_T_NZ");
    newRoutine(
        left_T_NZSafe.resetOdometry(),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, left_T_NZSafe, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
                new SequentialCommandGroup(new WaitCommand(0.60), superstructure.autoRunIntake()))
            .until(drive::isBeached)
            .handleInterrupt(() -> antiBeach(drive)),
        cmdWithAccuracy(drive, left_Safe_NZ_T),
        new ParallelCommandGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                new SequentialCommandGroup(
                        superstructure.autoRetractIntake().withTimeout(0.2),
                        superstructure.deployIntake().withTimeout(0.2))
                    .repeatedly()
                    .withTimeout(2.8),
                superstructure.autoRetractIntake())),
        new ParallelDeadlineGroup(
                cmdWithAccuracy(
                    drive, left_2nd_T_NZ, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
                new SequentialCommandGroup(new WaitCommand(1.50), superstructure.autoRunIntake()))
            .until(drive::isBeached)
            .handleInterrupt(() -> antiBeach(drive)),
        cmdWithAccuracy(drive, left_Safe_NZ_T),
        new ParallelCommandGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                new SequentialCommandGroup(
                        superstructure.autoRetractIntake().withTimeout(0.2),
                        superstructure.deployIntake().withTimeout(0.2))
                    .repeatedly()
                    .withTimeout(2.8),
                superstructure.autoRetractIntake())));
  }
}

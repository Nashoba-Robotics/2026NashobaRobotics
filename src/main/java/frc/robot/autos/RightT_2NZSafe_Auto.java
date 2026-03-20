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

public class RightT_2NZSafe_Auto extends AutoModeBase {
  public RightT_2NZSafe_Auto(Drive drive, Superstructure superstructure, AutoFactory factory) {
    super(factory, "RightT_2NZSafe_Auto");

    AutoTrajectory right_T_NZSafe = trajectory("Right_T_NZSafe");
    AutoTrajectory right_Safe_NZ_T = trajectory("Right_Safe_NZ_T");
    AutoTrajectory right_2nd_T_NZ = trajectory("Right_2nd_T_NZ");
    newRoutine(
        right_T_NZSafe.resetOdometry(),
        new ParallelDeadlineGroup(
            cmdWithAccuracy(
                drive, right_T_NZSafe, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
            new SequentialCommandGroup(new WaitCommand(0.60), superstructure.autoRunIntake())),
        cmdWithAccuracy(drive, right_Safe_NZ_T),
        new ParallelCommandGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                new SequentialCommandGroup(
                        superstructure.autoRetractIntake().withTimeout(0.2),
                        superstructure.deployIntake().withTimeout(0.2))
                    .repeatedly()
                    .withTimeout(2.4),
                superstructure.autoRetractIntake())),
        new ParallelDeadlineGroup(
            cmdWithAccuracy(
                drive, right_2nd_T_NZ, Units.Seconds.of(20.0), Units.Centimeters.of(5.0)),
            new SequentialCommandGroup(new WaitCommand(1.50), superstructure.autoRunIntake())),
        cmdWithAccuracy(drive, right_Safe_NZ_T),
        new ParallelCommandGroup(
            superstructure.autoShoot(),
            new SequentialCommandGroup(
                new WaitCommand(AutoConstants.kDelayIntakeRetract),
                new SequentialCommandGroup(
                        superstructure.autoRetractIntake().withTimeout(0.2),
                        superstructure.deployIntake().withTimeout(0.2))
                    .repeatedly()
                    .withTimeout(2.4),
                superstructure.autoRetractIntake())));
  }
}

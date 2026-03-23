// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.LoggedTunableNumber;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = true;
  public static final boolean disableHAL = false;

  public static final double loopTime = 0.02;

  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Hood {
    public static final String CANBUS = "rio";
    public static final int MOTOR_ID = 12;
    public static final int ENCODER_ID = 1;

    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final SensorDirectionValue ENCODER_DIRECTION =
        SensorDirectionValue.Clockwise_Positive;
    public static final double ENCODER_OFFSET = 0.0;
    public static final double ENCODER_DISCONTINUITY_POINT = 0.5;

    public static final double STATOR_LIMIT = 60.0;
    public static final double SUPPLY_LIMIT = 40.0;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/PID/kP", 10000.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/PID/kD", 250.0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/PID/kS", 3.0);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/PID/kV", 0.0);
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/PID/kA", 0.0);

    public static final LoggedTunableNumber POSITION_TOLERANCE =
        new LoggedTunableNumber("Hood/ToleranceDeg", 1.0);

    public static final double ROTOR_TO_SENSOR_GEAR_RATIO = 8.5;
    public static final double SENSOR_TO_MECHANISM_GEAR_RATIO = 17.0;
  }

  public static class Spindexer {
    public static final String CANBUS = "rio";
    public static final int MOTOR_ID = 14;

    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double STATOR_LIMIT = 60.0;
    public static final double SUPPLY_LIMIT = 40.0;

    public static final double GEAR_RATIO = 3.0;
  }

  public static class Intake {
    public static final String CANBUS = "rio";
    public static final int DEPLOY_MOTOR_ID = 16;
    public static final int ROLLER_LEADER_MOTOR_ID = 15;
    public static final int ROLLER_FOLLOWER_MOTOR_ID = 18;
    public static final int ENCODER_ID = 0;

    public static final InvertedValue DEPLOY_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue ROLLER_INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final SensorDirectionValue ENCODER_DIRECTION =
        SensorDirectionValue.Clockwise_Positive;
    public static final double ENCODER_OFFSET = -0.76171875;
    public static final double ENCODER_DISCONTINUITY_POINT = 0.85;

    public static final double DEPLOY_STATOR_LIMIT = 60.0;
    public static final double DEPLOY_SUPPLY_LIMIT = 40.0;

    public static final double ROLLER_STATOR_LIMIT = 80.0;
    public static final double ROLLER_SUPPLY_LIMIT = 60.0;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/PID/kP", 0.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/PID/kD", 0.0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/PID/kS", 0.0);
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/PID/kG", 0.0);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/PID/kV", 0.0);
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/PID/kA", 0.0);

    public static final Rotation2d GRAVITY_POSTION_OFFSET = Rotation2d.fromDegrees(0.0);

    public static final LoggedTunableNumber POSITION_TOLERANCE =
        new LoggedTunableNumber("Intake/DeployToleranceDeg", 3.0);

    public static final double ROLLER_GEAR_RATIO = 2.0;
    public static final double DEPLOY_ROTOR_TO_SENSOR_GEAR_RATIO = 45.0 * 16 / 18;
    public static final double DEPLOY_SENSOR_TO_MECHANISM_GEAR_RATIO = 2.0;
  }

  public static class Loader {
    public static final String CANBUS = "rio";
    public static final int LEFT_MOTOR_ID = 13;
    public static final int RIGHT_MOTOR_ID = 17;

    public static final InvertedValue LEFT_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue RIGHT_INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double STATOR_LIMIT = 80.0;
    public static final double SUPPLY_LIMIT = 60.0;

    public static final double GEAR_RATIO = 1.0;
  }

  public static class Shooter {
    public static final String CANBUS = "rio";
    public static final int SHOOTER_LEADER_ID = 8;
    public static final int SHOOTER_FOLLOWER1_ID = 9;
    public static final int SHOOTER_FOLLOWER2_ID = 10;
    public static final int SHOOTER_FOLLOWER3_ID = 11;

    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final double STATOR_LIMIT = 60.0;
    public static final double SUPPLY_LIMIT = 60.0;

    public static final LoggedTunableNumber kP =
        new LoggedTunableNumber("Shooter/Right/PID/kP", 10.0);
    public static final LoggedTunableNumber kD =
        new LoggedTunableNumber("Shooter/Right/PID/kD", 0.0);
    public static final LoggedTunableNumber kS =
        new LoggedTunableNumber("Shooter/Right/PID/kS", 5.2);
    public static final LoggedTunableNumber kV =
        new LoggedTunableNumber("Shooter/Right/PID/kV", 0.043);
    public static final LoggedTunableNumber kA =
        new LoggedTunableNumber("Shooter/Right/PID/kA", 0.0);

    public static final LoggedTunableNumber VELOCITY_TOLERANCE =
        new LoggedTunableNumber("Shooter/ToleranceRadsPerSec", 15.0);

    public static final double GEAR_RATIO = 1.35;
  }
}

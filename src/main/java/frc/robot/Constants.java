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

  public static class Climber {
    public static final String CANBUS = "rio";
    public static final int MOTOR_ID = 0;
    public static final int ENCODER_ID = 0;

    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final SensorDirectionValue ENCODER_DIRECTION =
        SensorDirectionValue.Clockwise_Positive;
    public static final double ENCODER_OFFSET = 0.0;
    public static final double ENCODER_DISCONTINUITY_POINT = 0.0;

    public static final double STATOR_LIMIT = 0.0;
    public static final double SUPPLY_LIMIT = 0.0;

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0;
    public static final double MOTION_MAGIC_ACCELERATION = 0.0;

    public static final Rotation2d FORWARD_SOFT_LIMIT = new Rotation2d(0.0);
    public static final Rotation2d REVERSE_SOFT_LIMIT = new Rotation2d(0.0);

    public static final double SENSOR_TO_MECHANISM_GEAR_RATIO = 0.0;
    public static final double ROTOR_TO_MECHANISM_GEAR_RATIO = 0.0;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("PID/Climber/kP", 0.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("PID/Climber/kD", 0.0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("PID/Climber/kS", 0.0);
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("PID/Climber/kG", 0.0);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("PID/Climber/kV", 0.0);
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("PID/Climber/kA", 0.0);

    public static final LoggedTunableNumber POSITION_TOLERANCE =
        new LoggedTunableNumber("PID/Climber/ToleranceDeg", 0.0);
  }

  public static class Hood {
    public static final String CANBUS = "rio";
    public static final int MOTOR_ID = 12;

    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final double STATOR_LIMIT = 60.0;
    public static final double SUPPLY_LIMIT = 60.0;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("PID/Hood/kP", 10000.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("PID/Hood/kD", 250.0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("PID/Hood/kS", 3.0);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("PID/Hood/kV", 0.0);
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("PID/Hood/kA", 0.0);

    public static final LoggedTunableNumber POSITION_TOLERANCE =
        new LoggedTunableNumber("PID/Hood/ToleranceDeg", 1.0);

    public static final double GEAR_RATIO = 144.5;
  }

  public static class Spindexer {
    public static final String CANBUS = "rio";
    public static final int MOTOR_ID = 14;

    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double STATOR_LIMIT = 40.0;
    public static final double SUPPLY_LIMIT = 40.0;

    public static final double GEAR_RATIO = 1.0;
  }

  public static class Intake {
    public static final String CANBUS = "rio";
    public static final int DEPLOY_MOTOR_ID = 16;
    public static final int ROLLER_MOTOR_ID = 15;

    public static final InvertedValue DEPLOY_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue ROLLER_INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double DEPLOY_STATOR_LIMIT = 80.0;
    public static final double DEPLOY_SUPPLY_LIMIT = 40.0;

    public static final double ROLLER_STATOR_LIMIT = 80.0;
    public static final double ROLLER_SUPPLY_LIMIT = 60.0;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("PID/Intake/kP", 460.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("PID/Intake/kD", 0.0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("PID/Intake/kS", 0.0);
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("PID/Intake/kG", 32.0);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("PID/Intake/kV", 0.0);
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("PID/Intake/kA", 0.0);

    public static final LoggedTunableNumber POSITION_TOLERANCE =
        new LoggedTunableNumber("PID/Intake/DeployToleranceDeg", 3.0);

    public static final double ROLLER_GEAR_RATIO = 30.0 / 14.0;
    public static final double DEPLOY_GEAR_RATIO = 23.0;

    public static final Rotation2d GRAVITY_POSTION_OFFSET = Rotation2d.fromDegrees(90.0);
  }

  public static class Loader {
    public static final String CANBUS = "rio";
    public static final int MOTOR_ID = 13;

    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double STATOR_LIMIT = 80.0;
    public static final double SUPPLY_LIMIT = 80.0;

    public static final double GEAR_RATIO = 1.0;
  }

  public static class Shooter {
    public static final String CANBUS = "rio";
    public static final int LEFT_SHOOTER_LEADER_ID = 8;
    public static final int LEFT_SHOOTER_FOLLOWER_ID = 9;
    public static final int RIGHT_SHOOTER_LEADER_ID = 10;
    public static final int RIGHT_SHOOTER_FOLLOWER_ID = 11;

    public static final InvertedValue LEFT_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue RIGHT_INVERTED = InvertedValue.Clockwise_Positive;

    public static final double STATOR_LIMIT = 80.0;
    public static final double SUPPLY_LIMIT = 80.0;

    public static final LoggedTunableNumber LEFT_kP =
        new LoggedTunableNumber("PID/Shooter/Left/kP", 10.0);
    public static final LoggedTunableNumber LEFT_kD =
        new LoggedTunableNumber("PID/Shooter/Left/kD", 0.0);
    public static final LoggedTunableNumber LEFT_kS =
        new LoggedTunableNumber("PID/Shooter/Left/kS", 5.2);
    public static final LoggedTunableNumber LEFT_kV =
        new LoggedTunableNumber("PID/Shooter/Left/kV", 0.043);
    public static final LoggedTunableNumber LEFT_kA =
        new LoggedTunableNumber("PID/Shooter/Left/kA", 0.0);

    public static final LoggedTunableNumber RIGHT_kP =
        new LoggedTunableNumber("PID/Shooter/Right/kP", 10.0);
    public static final LoggedTunableNumber RIGHT_kD =
        new LoggedTunableNumber("PID/Shooter/Right/kD", 0.0);
    public static final LoggedTunableNumber RIGHT_kS =
        new LoggedTunableNumber("PID/Shooter/Right/kS", 5.2);
    public static final LoggedTunableNumber RIGHT_kV =
        new LoggedTunableNumber("PID/Shooter/Right/kV", 0.043);
    public static final LoggedTunableNumber RIGHT_kA =
        new LoggedTunableNumber("PID/Shooter/Right/kA", 0.0);

    public static final LoggedTunableNumber VELOCITY_TOLERANCE =
        new LoggedTunableNumber("PID/Shooter/ToleranceRadsPerSec", 20.0);

    public static final double GEAR_RATIO = 1.5;
  }
}

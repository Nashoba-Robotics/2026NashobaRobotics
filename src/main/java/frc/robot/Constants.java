// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

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

    public static final Slot0Configs PID =
        new Slot0Configs().withKP(0.0).withKD(0.0).withKV(0.0).withKS(0.0).withKG(0.0);

    public static final double GEAR_RATIO = 0.0;
  }

  public static class Hood {
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

    public static final double SENSOR_TO_MECHANISM_GEAR_RATIO = 0.0;
    public static final double ROTOR_TO_MECHANISM_GEAR_RATIO = 0.0;

    public static final Slot0Configs PID =
        new Slot0Configs().withKP(0.0).withKD(0.0).withKS(0.0).withKV(0.0);

    public static final double GEAR_RATIO = 0.0;
  }

  public static class Hopper {
    public static final String CANBUS = "rio";
    public static final int MOTOR_ID = 0;

    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final double STATOR_LIMIT = 0.0;
    public static final double SUPPLY_LIMIT = 0.0;

    public static final Slot0Configs PID =
        new Slot0Configs().withKP(0.0).withKD(0.0).withKS(0.0).withKV(0.0);

    public static final double GEAR_RATIO = 0.0;
  }

  public static class Intake {
    public static final String CANBUS = "rio";
    public static final int DEPLOY_MOTOR_ID = 0;
    public static final int ROLLER_MOTOR_ID = 0;
    public static final int DEPLOY_ENCODER_ID = 0;

    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final SensorDirectionValue ENCODER_DIRECTION =
        SensorDirectionValue.Clockwise_Positive;
    public static final double ENCODER_OFFSET = 0.0;
    public static final double ENCODER_DISCONTINUITY_POINT = 0.0;

    public static final double STATOR_LIMIT = 0.0;
    public static final double SUPPLY_LIMIT = 0.0;

    public static final Slot0Configs PID =
        new Slot0Configs().withKP(0.0).withKD(0.0).withKS(0.0).withKV(0.0);

    public static final double GEAR_RATIO = 0.0;
  }

  public static class Loader {
    public static final String CANBUS = "rio";
    public static final int MOTOR_ID = 0;

    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final double STATOR_LIMIT = 0.0;
    public static final double SUPPLY_LIMIT = 0.0;

    public static final Slot0Configs PID =
        new Slot0Configs().withKP(0.0).withKD(0.0).withKV(0.0).withKS(0.0);

    public static final double GEAR_RATIO = 0.0;
  }

  public static class Shooter {
    public static final String CANBUS = "rio";
    public static final int MOTOR_ID = 0;

    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final double STATOR_LIMIT = 0.0;
    public static final double SUPPLY_LIMIT = 0.0;

    public static final Slot0Configs PID =
        new Slot0Configs().withKP(0.0).withKD(0.0).withKV(0.0).withKS(0.0);

    public static final double GEAR_RATIO = 0.0;
  }
}

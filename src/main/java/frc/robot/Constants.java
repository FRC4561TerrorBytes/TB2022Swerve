// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int FALCON_500_MAX_RPM = 6380;
    public static final int NEO_MAX_RPM = 5676;
    public static final int CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION = 2048;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 23;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(127.0); // FIXM E Measure and set front
                                                                                        // left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(77.0); // FIXM E Measure and set front
                                                                                        // right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 24;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(234.0); // FIXM E Measure and set back
                                                                                       // left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 21;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(97.1); // FIXM E Measure and set back
                                                                                       // right steer offset

    public static final int INTAKE_ROLLER_MOTOR = 12;
    public static final int PCM = 20;
    public static final int LEFT_SOLENOID = 1;
    public static final int RIGHT_SOLENOID = 0;

    public static final double INTAKE_SPEED = 0.6;
    public static final double FEEDER_SPEED = 0.5;

    public static final int FEEDER_LOWER_MOTOR = 11;
    public static final int FEEDER_MIDDLE_MOTOR = 10;
    public static final int FEEDER_UPPER_MOTOR = 9;
    public static final int UPPER_FEEDER_SENSOR = 0;

    public static final int LEFT_FLYWHEEL_MOTOR = 16;
    public static final int RIGHT_FLYWHEEL_MOTOR = 15;
    public static final int TURRET_MOTOR = 13;
    public static final int HOOD_MOTOR = 14;

    private static final boolean HOOD_MOTOR_SENSOR_PHASE = false;
    private static final boolean HOOD_INVERT_MOTOR = true;
    private static final double HOOD_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
    private static final double HOOD_MAX_RPM = FALCON_500_MAX_RPM;
    private static final double HOOD_KP = 1.0;
    private static final double HOOD_KI = 0.0;
    private static final double HOOD_KD = 0.0;
    private static final double HOOD_MECHANICAL_EFFICIENCY = 0.5;
    private static final double HOOD_TOLERANCE = 1.0;
    private static final double HOOD_LOWER_LIMIT = -800;
    private static final double HOOD_UPPER_LIMIT = HOOD_LOWER_LIMIT + 3900;
    private static final boolean HOOD_ENABLE_SOFT_LIMITS = true;
    private static final double HOOD_VELOCITY_RPM = FALCON_500_MAX_RPM / 60;
    private static final double HOOD_ACCELERATOIN_RPM_PER_SEC = HOOD_VELOCITY_RPM;
    private static final int HOOD_MOTION_SMOOTHING = 1;

    private static final boolean FLYWHEEL_INVERT_MOTOR = true;
    private static final double FLYWHEEL_MAX_RPM = NEO_MAX_RPM;
    private static final double FLYWHEEL_KP = 1.0;
    private static final double FLYWHEEL_KI = 0.0;
    private static final double FLYWHEEL_KD = 0.0;
    private static final double FLYWHEEL_MECHANICAL_EFFICIENCY = 0.5;
    private static final double FLYWHEEL_TOLERANCE = 1.0;
    private static final double FLYWHEEL_LOWER_LIMIT = 0;
    private static final double FLYWHEEL_UPPER_LIMIT = 0;
    private static final boolean FLYWHEEL_ENABLE_SOFT_LIMITS = false;
    private static final double FLYWHEEL_VELOCITY_RPM = NEO_MAX_RPM;
    private static final double FLYWHEEL_ACCELERATOIN_RPM_PER_SEC = NEO_MAX_RPM;
    private static final int FLYWHEEL_MOTION_SMOOTHING = 1;

    private static final boolean TURRET_INVERT_MOTOR = true;
    private static final double TURRET_MAX_RPM = NEO_MAX_RPM;
    private static final double TURRET_KP = 1.0;
    private static final double TURRET_KI = 0.0;
    private static final double TURRET_KD = 0.0;
    private static final double TURRET_MECHANICAL_EFFICIENCY = 0.5;
    private static final double TURRET_TOLERANCE = 1.0;
    private static final double TURRET_LOWER_LIMIT = 0;
    private static final double TURRET_UPPER_LIMIT = 0;
    private static final boolean TURRET_ENABLE_SOFT_LIMITS = false;
    private static final double TURRET_VELOCITY_RPM = NEO_MAX_RPM;
    private static final double TURRET_ACCELERATOIN_RPM_PER_SEC = NEO_MAX_RPM;
    private static final int TURRET_MOTION_SMOOTHING = 1;

    public static final TalonPIDConfig HOOD_MOTOR_CONFIG = new TalonPIDConfig(
            HOOD_MOTOR_SENSOR_PHASE,
            HOOD_INVERT_MOTOR,
            HOOD_TICKS_PER_ROTATION,
            HOOD_MAX_RPM,
            HOOD_KP,
            HOOD_KI,
            HOOD_KD,
            HOOD_MECHANICAL_EFFICIENCY,
            HOOD_TOLERANCE,
            HOOD_LOWER_LIMIT,
            HOOD_UPPER_LIMIT,
            HOOD_ENABLE_SOFT_LIMITS,
            HOOD_VELOCITY_RPM,
            HOOD_ACCELERATOIN_RPM_PER_SEC,
            HOOD_MOTION_SMOOTHING);

    public static final SparkPIDConfig FLYWHEEL_CONFIG = new SparkPIDConfig(
            FLYWHEEL_INVERT_MOTOR,
            FLYWHEEL_MAX_RPM,
            FLYWHEEL_KP,
            FLYWHEEL_KI,
            FLYWHEEL_KD,
            FLYWHEEL_MECHANICAL_EFFICIENCY,
            FLYWHEEL_TOLERANCE,
            FLYWHEEL_LOWER_LIMIT,
            FLYWHEEL_UPPER_LIMIT,
            FLYWHEEL_ENABLE_SOFT_LIMITS,
            FLYWHEEL_VELOCITY_RPM,
            FLYWHEEL_ACCELERATOIN_RPM_PER_SEC,
            FLYWHEEL_MOTION_SMOOTHING);

            public static final SparkPIDConfig TURRET_CONFIG = new SparkPIDConfig(
                TURRET_INVERT_MOTOR,
                TURRET_MAX_RPM,
                TURRET_KP,
                TURRET_KI,
                TURRET_KD,
                TURRET_MECHANICAL_EFFICIENCY,
                TURRET_TOLERANCE,
                TURRET_LOWER_LIMIT,
                TURRET_UPPER_LIMIT,
                TURRET_ENABLE_SOFT_LIMITS,
                TURRET_VELOCITY_RPM,
                TURRET_ACCELERATOIN_RPM_PER_SEC,
                TURRET_MOTION_SMOOTHING);

}

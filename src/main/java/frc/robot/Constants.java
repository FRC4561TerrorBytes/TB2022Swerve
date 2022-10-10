// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6;

    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 23;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(127.0); // FIXM E Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(77.0); // FIXM E Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 24;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(234.0); // FIXM E Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 21;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(97.1); // FIXM E Measure and set back right steer offset

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
}

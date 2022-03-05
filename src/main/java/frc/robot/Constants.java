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
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .441; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = .441; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 19; // FIXME Set Pigeon ID

    public static final int SHOOTER_MOTOR_FRONT = 31; // CAN ID of the front shooter motor
    public static final int SHOOTER_MOTOR_BACK = 32; // CAN ID of the back shooter motor

    public static final int INDEXER_MOTOR_FRONT = 41; // CAN ID of the roll intake motor
    public static final int INDEXER_MOTOR_BACK = 42; // CAN ID of the extension intake motor

    public static final int INTAKE_MOTOR_ROLLER = 45; // CAN ID of the roll intake motor
    public static final int INTAKE_MOTOR_EXTENSION = 46; // CAN ID of the extension intake motor

    public static final int MASTER_STEER_OFFSET = 0; // additional offset to true up robot travel direction

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 5; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(201.6129 + MASTER_STEER_OFFSET);

    // front
    // left steer offset
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 16; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 4; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(150.4578 + MASTER_STEER_OFFSET);

    // front
    // right steer offset
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 13; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 6; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(193.6203 + MASTER_STEER_OFFSET);

    // left
    // steer offset
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 15; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 7; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(171.6339 + MASTER_STEER_OFFSET);

    // right steer offset

    public static final int TURRET_MOTOR = 35; // CAN ID of the yaw turret motor
    public static final int ANGULAR_MOTOR = 36; // CAN ID of the angular (pitch) motor on the turret
    public static final int TURRET_SENSOR = 1; // Analog IN port for light beam sensor

    public static final int INDEXER_ENTRY = 2; // Sensor ID for the Indexer Entry
    public static final int INDEXER_MIDDLE = 3; // Sensor ID for the Indexer Middle
    public static final int INDEXER_EXIT = 4; // Sensor ID for the Indexer Exit
}

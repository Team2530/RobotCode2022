/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.libraries.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

public final class Constants {
    // ! NEED TO BE ACTUALLY SET
    // --------------------Motor Ports--------------------\\
    // DriveTrain Motors
    // ports set up for test drivetrain currently
    public static final int MOTOR_FL_DRIVE_PORT = 10;
    public static final int MOTOR_FR_DRIVE_PORT = 40;
    public static final int MOTOR_BL_DRIVE_PORT = 30;
    public static final int MOTOR_BR_DRIVE_PORT = 20;
    public static final int motor_revolver_port = 5;
    // TODO: Add real intake port
    public static final int motor_intake_port = 72;
    public static final int motor_climber_port = 80;
    public static final int motor_flywheel_port = 3;
    public static final int motor_turret_port = 4;
    public static final int motor_hood_port = 6;
    public static final int turret_encoder = 9;

    public enum DriveMotors {
        FL, FR, BL, BR;
    }

    // ----------Sensor Constants-----------\\
    public static final int ENCODER_TICKS_PER_REVOLUTION = 2048;
    public static final int gyroDrift = 5;
    // Mounting height, in inches
    public static final double sensor_Limelight_Height = 25;
    // TODO: I based this off of the gear tooth ratios, but don't know if that's
    // right
    public static final double HOOD_GEAR_RATIO = 11 / 72;
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;

    // ----------Driving Constants----------\\
    // The ratio between the encoder and the driven wheels
    public static final double DRIVE_GEAR_RATIO = 18.57;
    // ! Not diameter radius
    public static final double WHEEL_RADIUS = 6 * 2.54;
    public static final double DISTANCE_PER_PULSE = Constants.ENCODER_TICKS_PER_REVOLUTION * Math.PI
            * Math.pow(Constants.WHEEL_RADIUS, 2);
    public static final double kS = 0.761;
    public static final double kV = 0.0631;
    public static final double kA = 0.0095;
    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;
    public static final double kMaxSpeed = 2.7;
    public static final double kMaxAngularSpeed = 16 * Math.PI;
    // The amount of the max speed to start ramping with autonomous turning, from 0
    // to 1
    public static final double autoDriveMinRampTurnSpeed = 0.6;
    // The amount to shift the peak of the ramping with autonomous turning. -1 moves
    // the peak to the start, 0 leaves it in the middle, and 1 moves the peak to the
    // end
    public static final double autoDriveRampTurnOffset = -0.85;
    // The maximum speed of autonomous turning, from 0 to 1
    public static final double autoDriveMaxTurnSpeed = 0.6;
    // The acceptable distance from the target angle for autonomous turning, in
    // degrees
    public static final double autoDriveTurnTolerance = 5;
    public static final Gains PIDleftDrive = new Gains(0.439, 0, 0, 0, 0, 0);
    public static final Gains PIDrigthDrive = new Gains(0.439, 0, 0, 0, 0, 0);
    public static final Gains motor_Shooter = new Gains(1, 0, 0, 0, 0, 0);

    // public static final double ALIGN = 0.025;
    public static final double WHEEL_DISTANCE = Units.inchesToMeters(21);

    public static final double tol = 5;
    public static final int setPoint = 1;

    // ! TODO: This needs to be set
    public static final double maxVelocityMetersPerSecond = 2.7;
    // ! TODO: This needs to be set
    public static final double maxAccelerationMetersPerSecondSq = 6.47;
    public static final double autoVoltageConstraint = 9.5;

    // ----------Field Constants----------\\
    // The gravity on Earth (should be changed if we compete on the Moon)
    public static final double gravity = 9.81;
    // temp test value (in meters)
    public static final double target_Height = 105 * 2.54 / 100;
    // ? pounds?
    public static final double ball_Weight = 0.3125;

    // ----------Control (Joystick) Constants----------\\
    public static final double deadzone = 0.1;
    public static final int stickport1 = 0; // stick port for joystick 1 
    public static final int stickport2 = 1; // stick port for joystick 2 (tankdrive and things like that)
    public static final int xboxport = 0; // xbox controller port (couldn't find subsection so through it in here)

    // ----------Control (Shooting) Constants----------\\
    // From floor to center of opening, in meters.
    public static final double SHOOTER_HEIGHT = 20 * 2.54 / 100;
    // Moment of inertia
    public static final float I = 1;
    // In cm
    public static final double SHOOTER_WHEEL_RADIUS = 6 * 2.54;
    // Effective efficiency percentage
    public static final double eff = 0.8;
    // In meters
    public static final double MAX_SHOOTING_DISTANCE = 2.54;
    // In meters per second
    public static final int MAX_SHOOTING_VELOCITY = 20;
    // In degrees
    public static final int MIN_SHOOTING_ANGLE = 0;
    // In degrees
    public static final int MAX_SHOOTING_ANGLE = 30;
    // In cm
    public static final int IDEAL_SHOOTING_DISTANCE = 190;
    // In cm
    public static final double distanceTolerance = 10;
    // In degrees
    public static final double angleTolerance = 10;
}

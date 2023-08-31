package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.util.HIDHelper;

public class Constants {
    public static class DriveTrain{
        // Drivetrain
        public static final double DRIVE_XY_MULTIPLIER = 5.0;
        public static final double DRIVE_ROTATION_MULTIPLIER = 9.0;
        public static final double SLOW_DRIVE_XY_MULTIPLIER = DRIVE_XY_MULTIPLIER / 2.0;
        public static final double SLOW_DRIVE_ROTATION_MULTIPLIER = DRIVE_ROTATION_MULTIPLIER / 2.0;

        public static final double DRIVE_TURN_KP = 1.0;
        public static final double X_KP = 1.9;
        

        public static final double X_MOVE_MAX = 4;
        public static final double Y_MOVE_MAX = 4;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 7;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(265.1 - 180.0);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(292.236);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 12;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 11;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 4;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(219.7);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 9;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(33.8);

        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.02;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.02;

        public static final double DRIVE_ENCODER_TO_METERS = 40975.0;
        public static final double DRIVE_LL_CORRECT_KP = 1.0/15.0;
        public static final double DRIVE_GYRO_LOCK_KP = 1.0/20.0;

        public static final PIDController DRIVE_X_CONTROLLER = new PIDController(2.2, 0, 0);
        public static final PIDController DRIVE_Y_CONTROLLER = new PIDController(2.2, 0, 0);
        public static final PIDController DRIVE_ROTATION_CONTROLLER = new PIDController(6.0, 0, 0);

        public static final double SWERVE_MAX_VOLTAGE = 9.5;
        public static final double SWERVE_MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * SdsModuleConfigurations.MK4_L3.getDriveReduction() * SdsModuleConfigurations.MK4_L3.getWheelDiameter() * Math.PI;
        public static final double SWERVE_MAX_ANGULAR_VELOCITY = SWERVE_MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2.0, 2.0);

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            // Front left
            new Translation2d(0.41, 0.41),
            // Front right
            new Translation2d(0.41, -0.41),
            // Back left
            new Translation2d(-0.41, 0.41),
            // Back right
            new Translation2d(-0.41, -0.41));
            
        //Pigion ID
        public static final int PIGION_ID = 0;
    }

    public static class Lights {
        public static final int LIGHTS_ID = 0;
        public static final int NUM_LEDS = 200;
    }

    public static class Joysticks{
        // ### Joystick Constants ###
        public static final Joystick MASTER = new Joystick(0);
        public static final Joystick SECOND = new Joystick(1);
        public static final Joystick WHEEL = new Joystick(2);
        public static final XboxController XBOX = new XboxController(0);
        public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0.07, 0.65, -1, 0.4, 2);

        /**
         * The reason for these negative signs on the Y-axis
         * is because the sticks we use are designed for plane sims where pulling back on the stick sends you up
         */
        public static final HIDHelper.HIDConstants MASTER_STICK_SHIFTED = new HIDHelper.HIDConstants(MASTER, 0, 0.45, -1, 0.4, 2);
        public static final HIDHelper.HIDConstants SECOND_STICK = new HIDHelper.HIDConstants(SECOND, 0.01, -0.33, 0.99, 0.8, 2);  
        public static final HIDHelper.HIDConstants WHEEL_STICK = new HIDHelper.HIDConstants(WHEEL, 0, 2, 1, 1, 1, 2);  
    
        // Joystick deadzone
        public static final double XBOX_DEADZONE = 0.15;
        public static final double DEAD_ZONE = 0.05;
        public static final double OPEN_LOOP_FILTER = 0.3;
    }

    public static class Physical {
        // Physical Constants
        public static final double LOOPER_DT = 0.01;
        public static final double DRIVE_WHEEL_TRACK_WIDTH = .55;
        public static final double DRIVE_WHEEL_DIAMETER = 0.159; // m
        public static final double ROBOT_LINEAR_INERTIA = 55;  // kg TODO tune
        public static final double ROBOT_ANGULAR_INERTIA = 10;  // kg m^2 TODO tune
        public static final double ROBOT_ANGULAR_DRAG = 12.0;  // N*m / (rad/sec) TODO tune
        public static final double DRIVE_WHEEL_RADIUS = DRIVE_WHEEL_DIAMETER / 2.0;
        public static final double TRACK_SCRUB_FACTOR = 1.2;  //fudge factor for turning
    }
    public static class PathFollower {
        // Path following Constants
        public static final double ROBOT_MAX_VELOCITY = 1.5; // in/s
        public static final double ROBOT_MAX_ACCEL = 5; // in/s^2
        public static final double PATH_FOLLOWING_LOOKAHEAD = .6;
        public static final double DRIVETRAIN_UPDATE_RATE = 0.01;
        public static final double PATH_FOLLOWING_MAX_ACCELERATION = 60;
        public static final double ROBOT_MAX_VOLTAGE = 10.0; // V
        public static final double Path_Kx = 4.0;  //
        public static final double PATH_LOOK_AHEAD_TIME = 0.4;  // seconds to look ahead along the path for steering
        public static final double PATH_MIN_LOOK_AHEAD_DISTANCE = 24.0;  // inches
    }
    public static class Electrical {
        // Electrical Constants
        public static final double DRIVE_VCOMP = 10.0; //V
        public static final double DRIVE_ENCODER_PPR_HIGH_GEAR = 14200; //Empir through the magic of tape and wheel rotations HIGH GEAR
        public static final double DRIVE_ENCODER_PPR = 31828; //Empir through the magic of tape and wheel rotations LOW GEAR
        public static final double DRIVE_V_INTERCEPT = .621;  // V //1.6 for practice......................
        public static final double DRIVE_Kv = 4.716;  // rad/Vs 
        public static final double DRIVE_Ka = 56.179;  // rad/Vs^2
    }
}

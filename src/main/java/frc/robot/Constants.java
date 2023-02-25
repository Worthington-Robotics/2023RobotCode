// THIS IS COPIED AND SHOULD HAVE STUFF CHANGED

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;

public class Constants {
    // ### Device ID declarations ###

    // Drivetrain
    public static final int DRIVE_FRONT_LEFT_ID = 1;
    public static final int DRIVE_FRONT_RIGHT_ID = 3;
    public static final int DRIVE_BACK_LEFT_ID = 2;
    public static final int DRIVE_BACK_RIGHT_ID = 4;

    public static final int DRIVE_TRANSMISSION_FORWARD = 1;
    public static final int DRIVE_TRANSMISSION_REVERSE = 0;
    
    // Arm and Turret Constants

    // Arm Consants - IDs
    public static final int ARM_TURRET_ID = 9;
    public static final int ARM_ARM_M_ID = 10;
    public static final int ARM_EXTENSION_ID = 11;
    public static final int ARM_ARM_S_ID = 12;
    
    public static final int CTRE_PCM_ID = 0;
    public static final int ARM_GRABBER_FWD_CHANNEL = 5;
    public static final int ARM_GRABBER_REV_CHANNEL = 4;

    //Manipulator
    public static final int INTAKE_WHEEL_ID = 5;
    public static final int MANIPULATOR_TOF_ID = 1;
    public static final double INTAKE_POWER = .7;
    public static final double ANYTHING_OUT_POWER = -.7;

    public static final double PIVOT_INCREMENT = 0.1;
    public static final double MAX_PIVOT_POWER = 0.7;
    public static final double MIN_PIVOT_POWER = 0;
    
    // Arm Constants - Measurements
    public static final double PIVOT_ENCODER_PER_DEGREE = 416.31;
    public static final double TURRET_ENCODER_PER_DEGREE = 218.56;
    public static final double ENCODER_PER_INCH = 3904.5;

    // Arm Constants - PID 
    public static final double ARM_PIVOT_KP = 1.9; // TODO: Adjust all kps and mins and maxs for arm
    public static final double PIVOT_ANGLE_ACCEPTANCE = 2.0;
    public static final double PIVOT_MIN_TIME = 1.0;

    public static final double TURRET_KP = 1.0 / 250.0;
    public static final double TURRET_ANGLE_ACCEPTANCE = 2.0;
    public static final double TURRET_MIN_TIME = 1.5;

    public static final double ARM_EXTENSION_KP = 1.0 / 100.0;
    public static final double EXTENSION_DISTANCE_ACCEPTED = 2.0;
    public static final double EXTENSION_MIN_TIME = 1.5;
    
    
    // Arm Constants - Safety
    public static final double PIVOT_WARNING_ANGLE = 5.0; // Degrees(not ticks) from min or max when arm slows
    public static final double MAX_PIVOT = 75.0; // TODO: Actual Value
    public static final double MIN_PIVOT = 0.0; // Assume that arm moves slow enough to completely stop and reset when limit switch activated
    public static final double PIVOT_MAX_SPEED = 0.7;
    public static final double PIVOT_MIN_SPEED = 0.1;

    public static final double EXTENSION_WARNING_DISTANCE = 5.0; // distance from min or max when arm slows, in inches
    public static final double MAX_ARM_LENGTH = 50.0; // TODO: Actual value
    public static final double MIN_ARM_LENGTH = 1.0;
    public static final double EXTENSION_MAX_SPEED = 0.5;
    public static final double EXTENSION_MIN_SPEED = 0.1;

    public static final double TURRET_WARNING_DISTANCE = 5.0; // Degree
    public static final double TURRET_MIN_ANGLE = -90.0;
    public static final double TURRET_MAX_ANGLE = 90.0;
    public static final double TURRET_MAX_SPEED = 0.75;
    public static final double TURRET_MIN_SPEED = 0.05;

    // ### Device ID declarations ###

    // Superstructure
    // TODO: Change these values to actual IDs
    public static final int BACKSTOP_TOF_ID = 0;
    public static final int WRIST_MOTOR_ID = 5;
    public static final int INTAKE_MOTOR_ID = 7;
    
    //Pigion ID
    public static final int PIGION_ID = 1;

    // Pigeon ID
    public static final int PIGEON_ID = 1;


    // ### Joystick Constants ###
    public static final Joystick MASTER = new Joystick(0);
    public static final Joystick SECOND = new Joystick(1);
    public static final Joystick WHEEL = new Joystick(2);
    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0.07, 0.65, -1, 0.4, 2); 

    /**
     * The reason for these negative signs on the Y-axis
     * is because the sticks we use are designed for plane sims where pulling back on the stick sends you up
     */
    public static final HIDHelper.HIDConstants MASTER_STICK_SHIFTED = new HIDHelper.HIDConstants(MASTER, 0, 0.45, -1, 0.4, 2);
    public static final HIDHelper.HIDConstants SECOND_STICK = new HIDHelper.HIDConstants(SECOND, 0.01, -0.33, 0.99, 0.8, 2);  
    public static final HIDHelper.HIDConstants WHEEL_STICK = new HIDHelper.HIDConstants(WHEEL, 0, 2, 1, 1, 1, 2);  

    // ### Intake tuned values ###

    // The target distance where the intake backstop TOF will report the game piece as being ready
    public static final double INTAKE_DISTANCE = 10.0d;

    // Solenoid IDs
    public static final int INTAKE_SOLINIOD_REVERSE = 2;
    public static final int INTAKE_SOLINIOD_FORWARD = 3;
    public static final int INTAKE_PNEUMATICS_ID = 0;

    // ### Drivetrain tuned values ###

    // Conversion factor from drive ticks to inches
    public static final double TICKS_PER_INCH = 1695;

    // Joystick deadzone
    public static final double DEAD_ZONE = 0.05;
    public static final double OPEN_LOOP_FILTER = 0.3;
    
  

    //Lights Constants
    public static final int LIGHTS_ID = 0;
    public static final int LIGHTS_LED_COUNT = 60;
    
    // Angle PID
    public static final double TURN_KP = 1.0 / 120.0;
    public static final double ANGLE_ACCEPTANCE = 1.8;
    public static final double ANGLE_PID_MINIMUM_TIME = 0.06;
    public static final double DRIVE_TURN_MINIMUM_SPEED = 0.08;
    public static final double DRIVE_TURN_MAXIMUM_SPEED = 0.5;

    // Move forward
    public static final double DRIVE_FORWARD_ACCEPTED_ERROR = 2000.0;
    public static final double DRIVE_FORWARD_MINIMUM_SPEED = 0.09;
    public static final double DRIVE_FORWARD_MAXIMUM_SPEED = 0.8;
    public static final double DRIVE_FORWARD_MINIMUM_TIME = 0.01;
    public static final double DRIVE_FORWARD_KP = 1.0 / 100000.0;
    public static final double DRIVE_FORWARD_KD = 0.0 / 10000.0;
    public static final double DRIVE_FORWARD_D_FILT = 2.5;
    // Heading correction when moving forward
    public static final double DRIVE_FORWARD_HEADING_KP = 1.0 / 300.0;

    // Auto level
    public static final double DRIVE_LEVEL_KP = 1.0 / 90.0;
    public static final double DRIVE_LEVEL_KD = 1.0 / 1.0;
    public static final double DRIVE_LEVEL_D_FILTER = 0.2;
    public static final double DRIVE_LEVEL_MAX_SPEED = 0.6;
    // Correction for pigeon pitch inaccuracy
    public static final double DRIVE_LEVEL_ZERO = -2.5;



    // ### Constants kept around for compatability with library code ###
    
    // DEBUG AND TESTING flags
    public static boolean WHEELS = true;
    public static final boolean RAMPUP = false;
    public static final boolean ENABLE_MP_TEST_MODE = false;
    public static final double MP_TEST_SPEED = 2; //m/s

    public static double LOOPER_DT = 0.01;   
    
    // Physical Constants
    public static final double DRIVE_WHEEL_TRACK_WIDTH = .55;
    public static final double DRIVE_WHEEL_DIAMETER = 0.159; // m
    public static final double ROBOT_LINEAR_INERTIA = 55;  // kg TODO tune
    public static final double ROBOT_ANGULAR_INERTIA = 10;  // kg m^2 TODO tune
    public static final double ROBOT_ANGULAR_DRAG = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double DRIVE_WHEEL_RADIUS = DRIVE_WHEEL_DIAMETER / 2.0;
    public static final double TRACK_SCRUB_FACTOR = 1.2;  //fudge factor for turning

    // Path following Constants
    public static final double ROBOT_MAX_VELOCITY = 1.5; // in/s
    public static final double ROBOT_MAX_ACCEL = 5; // in/s^2
    public static final double PATH_FOLLOWING_LOOKAHEAD = .6;
    public static final double DRIVETRAIN_UPDATE_RATE = LOOPER_DT;
    public static final double PATH_FOLLOWING_MAX_ACCELERATION = 60;
    public static final double ROBOT_MAX_VOLTAGE = 10.0; // V
    public static final double Path_Kx = 4.0;  //
    public static final double PATH_LOOK_AHEAD_TIME = 0.4;  // seconds to look ahead along the path for steering
    public static final double PATH_MIN_LOOK_AHEAD_DISTANCE = 24.0;  // inches

    // Electrical Constants
    public static final double DRIVE_VCOMP = 10.0; //V
    public static final double DRIVE_ENCODER_PPR_HIGH_GEAR = 14200; //Empir through the magic of tape and wheel rotations HIGH GEAR
    public static final double DRIVE_ENCODER_PPR = 31828; //Empir through the magic of tape and wheel rotations LOW GEAR
    public static final double DRIVE_V_INTERCEPT = .621;  // V //1.6 for practice......................
    public static final double DRIVE_Kv = 4.716;  // rad/Vs 
    public static final double DRIVE_Ka = 56.179;  // rad/Vs^2

    // PID Constants
    public static final double ANGLE_KP = 0.024; // 0.065;
    public static final double ANGLE_KI = 0; // 0.00125;
    public static final double ANGLE_KD = 0; // 0.1
    public static final double ANGLE_PID_EPISLON = 1;

    public static final double DRIVE_RIGHT_KP = 0;//.25;
    public static final double DRIVE_RIGHT_KI = 0.0;
    public static final double DRIVE_RIGHT_KD = 0;//5; // 20 for practice bot
    public static final double DRIVE_RIGHT_KF = 0.065; //.485

    public static final double DRIVE_LEFT_KP = 0;//.25; // .0885
    public static final double DRIVE_LEFT_KI = 0.0; //NO INTEGRAL it masks deeper problems
    public static final double DRIVE_LEFT_KD = 0;//5; //20 for practice
    public static final double DRIVE_LEFT_KF = 0.065;


    public static final int greenH = 120;
    public static final int blueH = 180;
    public static final int error = 29;
    public static final int satLimit = 80;
    public static final int valLimit = 80;

    /**
     * Deadzone Constants
     */
    public static final double deadZone = 0.05;

}

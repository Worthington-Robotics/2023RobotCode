// THIS IS COPIED AND SHOULD HAVE STUFF CHANGED

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;

public class Constants {
    // Drivetrain Constants
    public static final int DRIVE_FRONT_LEFT_ID = 1;
    public static final int DRIVE_FRONT_RIGHT_ID = 3;
    public static final int DRIVE_BACK_LEFT_ID = 2;
    public static final int DRIVE_BACK_RIGHT_ID = 4;
    //Arm Turret Constants
    public static final int ARM_TURRET_ID = 9;
    public static final int ARM_ARM_M_ID = 10;
    public static final int ARM_EXTENTION_ID = 11;
    public static final int ARM_ARM_S_ID = 12;
    
    public static final int CTRE_PCM_ID = 0;
    public static final int ARM_GRABBER_FWD_CHANNEL = 2;
    public static final int ARM_GRABBER_REV_CHANNEL = 3;

    public static final double ENCODER_PER_DEGREE = 1000.0;
    public static final double ENCODER_PER_INCH = 750.0;
    public static final double ARM_POWER_KP = .8;
    public static final double LEVER_LENGTH_KP = 1 / 30.0;

    // ### Device ID declarations ###

    // Superstructure
    // TODO: Change these values to actual IDs
    public static final int BACKSTOP_TOF_ID = 0;
    public static final int INTAKE_LEFT_WHEEL_ID = 5;
    public static final int INTAKE_RIGHT_WHEEL_ID = 7;
    public static final int INTAKE_CONVEYOR_ID = 6;
    public static final int INTAKE_SPINNER_ID = 8;
    
    //Pigion ID
    public static final int PIGION_ID = 1;

    //Talon SRX & FX IDs
    public static final int SHOOTER_FLYWHEEL_LEFT = 5;
    public static final int SHOOTER_FLYWHEEL_RIGHT = 6;

    public static final int CLIMBING_WINCHR_ID = 3;
    public static final int CLIMBING_WINCHL_ID = 4;

    public static final int ID_SUPER_DELIVERY_WHEEL = 7;
    public static final int ID_SUPER_INDEX1 = 8;
    public static final int ID_SUPER_INDEX2 = 9;
    public static final int ID_SUPER_INDEX3 = 10;
    public static final int ID_SUPER_INTAKE = 11;

    public static final int TURRET_CONTROL = 12;
    public static final int CAM_PWM_CHANNEL = 1;

    // TOF IDs
    public static final int ID_SUPER_TOF1 = 1;
    public static final int ID_SUPER_TOF2 = 2;
    public static final int ID_SUPER_TOF3 = 3;
    public static final int ID_SUPER_TOF4 = 4;
    public static final int ID_SUPER_TOF5 = 5;

    // ### Joystick Constants ###
    public static final Joystick MASTER = new Joystick(0);
    public static final Joystick SECOND = new Joystick(1);
    public static final Joystick WHEEL = new Joystick(2);
    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0.05, 0.65, -1, 0.4, 2); 

    /**
     * The reason for these negative signs on the Y-axis
     * is because the sticks we use are designed for plane sims where pulling back on the stick sends you up
     */
    public static final HIDHelper.HIDConstants MASTER_STICK_SHIFTED = new HIDHelper.HIDConstants(MASTER, 0, 0.45, -1, 0.4, 2);
    public static final HIDHelper.HIDConstants SECOND_STICK = new HIDHelper.HIDConstants(SECOND, 0.01, -0.33, 0.99, 0.8, 2);  
    public static final HIDHelper.HIDConstants WHEEL_STICK = new HIDHelper.HIDConstants(WHEEL, 0, 2, 1, 1, 1, 2);  

    // ### Intake tuned values ###

    // The target distance where the intake backstop TOF will report the game piece as being ready
    public static final double INTAKE_BACKSTOP_DISTANCE = 10.0d;
    // Multipliers for how much of the power each motor of the intake gets
    public static final double CONVEYER_BELT_MULTIPLIER = 2;
    public static final double INTAKE_WHEEL_SPINNER_MULTIPLIER = .5;
    public static final double SIDE_WHEELS_MULTIPLIER = 2;
    // Constants for how much power each type of intake uses
    public static final double CONE_IN_POWER = 1;
    public static final double CUBE_IN_POWER = .5; 
    public static final double ANYTHING_OUT_POWER = -1;
    // Solenoid IDs
    public static final int INTAKE_SOLINIOD_REVERSE = 4;
    public static final int INTAKE_SOLINIOD_FORWARD = 5;
    public static final int INTAKE_PNEUMATICS_ID = 0;

    // ### Drivetrain tuned values ###

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
    public static final double ANGLE_KP = -0.024; // 0.065;
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

    // Limelight Constants
    public static final double LIMELIGHT_DEG_FOV = 0.0; //TODO CALCULATE FOV
    public static final double fov = 0;
    public static final int redH1 = 0;
    public static final int redH2 = 360;
    public static final int yellowH = 60;
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

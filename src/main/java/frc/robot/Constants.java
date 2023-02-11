package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;
import java.io.IOException;
import java.nio.file.Files;

public class Constants {
    public static int DEBUG_LEVEL = 0;

    // Read the debug level from the debug file on the robot
    public static void readDebugLevel() {
        Path debugPath = Filesystem.getDeployDirectory().toPath().resolve("debug.txt");
        try {
            String contents = new String(Files.readAllBytes(debugPath));
            DEBUG_LEVEL = Integer.parseInt(contents);
        } catch (IOException _err) {
            DEBUG_LEVEL = 0;
        }
    }

    //Drivetrain Constants
    public static final int DRIVE_FORWARD_LEFT_ID = 0;
    public static final int DRIVE_FORWARD_RIGHT_ID = 0;
    public static final int DRIVE_BACK_LEFT_ID = 0;
    public static final int DRIVE_BACK_RIGHT_ID = 0;


    /**
     * device ID declarations ---------------------------------
     */
    // Superstructure
    // TODO: Change these values to actual IDs
    public static final int BACKSTOP_TOF_ID = 0;
    public static final int INTAKE_LEFT_WHEEL_ID = 0;
    public static final int INTAKE_RIGHT_WHEEL_ID = 0;
    public static final int INTAKE_CONVEYOR_ID = 0;
    public static final int INTAKE_SPINNER_ID = 0;

    //Joystick Constants
    public static final Joystick MASTER = new Joystick(0);
    public static final Joystick SECOND = new Joystick(1);
    public static final Joystick WHEEL = new Joystick(2);
    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0, 0.65, -1, 0.4, 2); 
    //The reason for these negative signs on the Y-axis
    // is because the sticks we use are designed for plane sims where pulling back on the stick sends you up
    public static final HIDHelper.HIDConstants MASTER_STICK_SHIFTED = new HIDHelper.HIDConstants(MASTER, 0, 0.45, -1, 0.4, 2);
    public static final HIDHelper.HIDConstants SECOND_STICK = new HIDHelper.HIDConstants(SECOND, 0.01, -0.33, 0.99, 0.8, 2);  
    public static final HIDHelper.HIDConstants WHEEL_STICK = new HIDHelper.HIDConstants(WHEEL, 0, 2, 1, 1, 1, 2);  

    // ### Intake tuned values ###

    // The target distance where the intake backstop TOF will report the
    // game piece as being ready
    public static final double INTAKE_BACKSTOP_DISTANCE = 10.0d;
    // Multipliers for how much of the power each motor of the intake gets
    public static final double CONVEYER_BELT_MULTIPLIER = 0.125;
    public static final double INTAKE_WHEEL_SPINNER_MULTIPLIER = .5;
    public static final double SIDE_WHEELS_MULTIPLIER = .25;
    //Constants for how much power each type of intake uses
    public static final double CONE_IN_POWER = 1;
    public static final double CUBE_IN_POWER = .5; 
    public static final double ANYTHING_OUT_POWER = -1;

    /* 
        Turret & flywheel tuned values ------------------------
    */
    public static final double LIMELIGHT_HIGHT = 23;
    public static final double LIMELIGHT_PITCH = 31;
    public static final double TURRET_MAX_SPEED = .15;
    public static final double TURRET_ANGLE_KP = .45; // SAFE .3
    public static final double TURRET_ANGLE_KI = 0.001; // SAFE .001
    public static final double TURRET_ANGLE_KD = 20; // SAFE 12
    public static final double TURRET_OFFSET = .75; //1.25 VISION FINE TUNING 25ft
    public static final double TURRET_DEGREES_TO_TICKS = 85.26;
    public static final double TURRET_LOCKON_DELTA = .5;

    public static double TURRET_LEFT_FLY_KP = 0.08; //0.08
    public static double TURRET_LEFT_FLY_KD = 0.0; //0.00
    public static double TURRET_LEFT_FLY_KF = 0.048; //0.048

    public static double VOLTAGE_COMP_FLYWHEEL = 10;
    public static final double FLYWHEEL_DELTA_AMPS = 2.5;

    public static final double FLYWHEEL_RPM_PER_IN = 4.4; //4.4
    public static final double FLYWHEEL_BASE_RPM = 4000; //4000
    public static final double FLYWHEEL_SPINUP_TIME = 75; //10 ms 
    public static final double FLYWHEEL_IDLE_RPM = 4600; //RPM
    public static final double FLYWHEEL_MAX_RPM = 6400; //RPM
    public static final int FLYWHEEL_MIN_RPM_OFFSET = -400;
    public static final int FLYWHEEL_MAX_RPM_OFFSET = 600;
    public static final int FLYWHEEL_OFFSET_RPM_INCREMENT = 100;
    public static final double FLYWHEEL_TP100MS = 3.413;

    public static final int CAM_ANGLE_HIGH = 180;
    public static final int CAM_ANGLE_MED = 90;
    public static final int CAM_ANGLE_LOW = 30;

    //Turret Encoder Limits
    public static final int leftTurretLimit = -7400;
    public static final int rightTurretLimit = 7400;

    /**
     *   Drivetrain tuned values ------------------------------
     */

    //DEBUG AND TESTING flags
    public static boolean WHEELS = true;
    public static final boolean RAMPUP = false;
    public static final boolean ENABLE_MP_TEST_MODE = true;
    public static final double MP_TEST_SPEED = 2; //m/s

    public static double LOOPER_DT = 0.01;   
    
    //Physical Constants
    public static final double DRIVE_WHEEL_TRACK_WIDTH = .55;
    public static final double DRIVE_WHEEL_DIAMETER = 0.159; // m
    public static final double ROBOT_LINEAR_INERTIA = 55;  // kg TODO tune
    public static final double ROBOT_ANGULAR_INERTIA = 10;  // kg m^2 TODO tune
    public static final double ROBOT_ANGULAR_DRAG = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double DRIVE_WHEEL_RADIUS = DRIVE_WHEEL_DIAMETER / 2.0;
    public static final double TRACK_SCRUB_FACTOR = 1.2;  //fudge factor for turning

    //Path following Constants
    public static final double ROBOT_MAX_VELOCITY = 1.5; // in/s
    public static final double ROBOT_MAX_ACCEL = 5; // in/s^2
    public static final double PATH_FOLLOWING_LOOKAHEAD = .6;
    public static final double DRIVETRAIN_UPDATE_RATE = LOOPER_DT;
    public static final double PATH_FOLLOWING_MAX_ACCELERATION = 60;
    public static final double ROBOT_MAX_VOLTAGE = 10.0; // V
    public static final double Path_Kx = 4.0;  //
    public static final double PATH_LOOK_AHEAD_TIME = 0.4;  // seconds to look ahead along the path for steering
    public static final double PATH_MIN_LOOK_AHEAD_DISTANCE = 24.0;  // inches

    //Electrical Constants
    public static final double DRIVE_VCOMP = 10.0; //V
    public static final double DRIVE_ENCODER_PPR_HIGH_GEAR = 14200; //Empir through the magic of tape and wheel rotations HIGH GEAR
    public static final double DRIVE_ENCODER_PPR = 31828; //Empir through the magic of tape and wheel rotations LOW GEAR
    public static final double DRIVE_V_INTERCEPT = .621;  // V //1.6 for practice......................
    public static final double DRIVE_Kv = 4.716;  // rad/Vs 
    public static final double DRIVE_Ka = 56.179;  // rad/Vs^2

    //PID Constants
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

    /**
     * Color Wheel Constants ---------------------------
     */
    //encoder constants
    public static final double COLOR_ENCODER_CPR = 1024;

    public static final double COLOR_WHEEL_ROTATION_DISTANCE = 3.2 * (16 * Math.PI);

    public static final double COLOR_WHEEL_SPINNER_DIA = 2.0;

    public static final double COLOR_WHEEL_KF = 0.0; //TODO TUNE PID
    public static final double COLOR_WHEEL_KP = 0.0;
    public static final double COLOR_WHEEL_KI = 0.0;
    public static final double COLOR_WHEEL_KD = 0.0;
    public static final double COLOR_WHEEL_VCOMP = 0.0;    

    //Limelight Constants
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
     * Superstructure constants
     */
    public static final double DEMAND_STOP = 0;

    public static final double SUPER_DEMAND_INTAKE_MANUAL = 1; // FIXME WARNING: MUST BE DIFFERENT FROM THE DEFAULT INTAKE DEMAND (unless you want me to implement a boolean for manual control)
    public static final double SUPER_DEMAND_SHOOT = 1;
    
    /**
     * Climber constants
     */
    public static final double CLIMBER_SHOOTER_REQMT = 90;
    public static final double CLIMBER_EPSILON_CONST = 10;
}



/*----------------------------------------------------------------------------*/
/* Copyright (c) 1892-1893 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.loops.Looper;
import frc.lib.models.DriveTrajectoryGenerator;
import frc.lib.statemachine.StateMachine;
import frc.robot.subsystems.*;
import frc.robot.autos.AutoChooser;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Arm.ArmPose;
import frc.lib.statemachine.Action;
import frc.robot.actions.drive.TeleopLevelAction;
import frc.robot.actions.drive.GyroLockAction;
import frc.robot.actions.drive.SetPositionAction;
import frc.robot.actions.drive.DriveTurnActionLimelight;
import frc.robot.actions.drive.GearChangeAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.CycleArmAction;
import frc.robot.actions.arm.PinToggleAction;
import frc.robot.actions.arm.PivotMoveAction;
import frc.robot.actions.arm.TurretHoldAction;
import frc.robot.actions.manipulator.MoveWristAction;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private SubsystemManager manager;
    private Looper enabledLooper, disabledLooper;



    // Input bindings
    private JoystickButton driveGearButton = new JoystickButton(Constants.MASTER, 1);
    private JoystickButton intakeButton = new JoystickButton(Constants.MASTER, 2);
    private JoystickButton intakeReverseButton = new JoystickButton(Constants.MASTER, 3);
    private JoystickButton autoLevelButton = new JoystickButton(Constants.MASTER, 4);
    // private JoystickButton limelightRotateButton = new JoystickButton(Constants.MASTER, 5);
    private JoystickButton resetPoseButton = new JoystickButton(Constants.MASTER, 6);
    private JoystickButton gyroLockButton = new JoystickButton(Constants.MASTER, 7);


    private JoystickButton stowButton = new JoystickButton(Constants.SECOND, 5);
    private JoystickButton unstowButton = new JoystickButton(Constants.SECOND, 6);
    private JoystickButton cubePickupButton = new JoystickButton(Constants.SECOND, 12);
    //private JoystickButton cubePickupButton = new JoystickButton(Constants.SECOND, 9);
    private JoystickButton cubeDropButton = new JoystickButton(Constants.SECOND, 10);
    private JoystickButton cycleButton = new JoystickButton(Constants.SECOND, 11);
    private JoystickButton coneDropButton = new JoystickButton(Constants.SECOND, 8);
    private JoystickButton wristUpButton = new JoystickButton(Constants.SECOND, 3);
    private JoystickButton wristDownButton = new JoystickButton(Constants.SECOND, 4);
    private JoystickButton turretHold = new JoystickButton(Constants.SECOND, 1);


    //private JoystickButton pivotDownHighButton = new JoystickButton(Constants.SECOND, 3);
    //private JoystickButton pivotUpHighButton = new JoystickButton(Constants.SECOND, 4);
   // private JoystickButton pivotDownSlowButton = new JoystickButton(Constants.SECOND, 7);
   // private JoystickButton pivotUpSlowButton = new JoystickButton(Constants.SECOND, 8);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        initButtons();
        CommandScheduler.getInstance().enable();
        manager = new SubsystemManager(
            Arrays.asList(
                Manipulator.getInstance(),
                PoseEstimator.getInstance(),
                Arm.getInstance(),
                DriveTrain.getInstance(),
                Lights.getInstance()
            ),
            true
        );

        enabledLooper = new Looper();
        disabledLooper = new Looper();
        Manipulator.getInstance().resetManipulatorEncoder();

        // Register the looper threads to the manager to use for enabled and disabled
        manager.registerEnabledLoops(enabledLooper);
        manager.registerDisabledLoops(disabledLooper);

        // Add any additional logging sources for capture
        manager.addLoggingSource(Arrays.asList(StateMachine.getInstance()));

        AutoChooser.getInstance().logAuto();
        AutoChooser.getInstance().printList();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        manager.outputTelemetry();  
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        enabledLooper.stop();

        //StateMachine.getInstance().assertStop();

        disabledLooper.start();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        disabledLooper.stop();
        enabledLooper.start();
        Arm.getInstance().clearPin();
        AutoChooser.getInstance().run_from_selection();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        disabledLooper.stop();

        // Reset anything here
        DriveTrain.getInstance().setOpenLoop();
        enabledLooper.start();
        Arm.getInstance().clearPin();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        disabledLooper.stop();

        Arm.getInstance().setPin();

        enabledLooper.start();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {}

    public void initButtons() {
        driveGearButton.whileTrue(Action.toCommand(new GearChangeAction()));
        intakeReverseButton.whileTrue(Action.toCommand(new RunIntakeAction(Constants.ANYTHING_OUT_POWER)));
        intakeButton.whileTrue(Action.toCommand(new RunIntakeAction(Constants.INTAKE_POWER)));
        autoLevelButton.whileTrue(Action.toCommand(new TeleopLevelAction()));
        resetPoseButton.onTrue(Action.toCommand(new SetPositionAction(0, 0, 0)));
        gyroLockButton.whileTrue(Action.toCommand(new GyroLockAction()));


        //Copy and paste armPoseAction line and change the armPose to the next arm Pose
        stowButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.STOWN)));
        unstowButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.UNSTOW)));
        cubeDropButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.CUBE_MID)));
        coneDropButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.CONE_MID)));
        cubePickupButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.INTAKE)));
        wristUpButton.whileTrue(Action.toCommand(new MoveWristAction(-.33)));
        wristDownButton.whileTrue(Action.toCommand(new MoveWristAction(.33)));
        turretHold.whileTrue(Action.toCommand(new TurretHoldAction()));
        cycleButton.whileTrue(Action.toCommand(new CycleArmAction()));
    }
}

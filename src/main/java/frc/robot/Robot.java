/*----------------------------------------------------------------------------*/
/* Copyright (c) 1892-1893 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.loops.Looper;
import frc.lib.statemachine.StateMachine;
import frc.lib.util.ReflectingLogger;
import frc.robot.subsystems.*;
import frc.robot.autos.AutoChooser;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Arm.ArmPose;
import frc.lib.statemachine.Action;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.vision.SetPipelineAction;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.CycleArmAction;
import frc.robot.actions.drive.DriveSwitchRobotMode;
import frc.robot.actions.drive.DriveZeroGyro;
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
    private JoystickButton intakeButton = new JoystickButton(Constants.XBOX, 1);
    private JoystickButton intakeReverseButton = new JoystickButton(Constants.XBOX, 2);
    //private JoystickButton limelightPipeButton = new JoystickButton(Constants.MASTER, 5);
   // private JoystickButton unStowButton = new JoystickButton(Constants.MASTER, 3);
    private JoystickButton cycleButton = new JoystickButton(Constants.XBOX, 7);
    private POVButton zeroPoseButton = new POVButton(Constants.XBOX, 180);

    private JoystickButton toggleDriveModeButton = new JoystickButton(Constants.XBOX, 5);
    private JoystickButton resetGyroButton = new JoystickButton(Constants.XBOX, 6);

    private JoystickButton slideButton = new JoystickButton(Constants.SECOND, 2);
    private JoystickButton intakeSpitButton = new JoystickButton(Constants.SECOND, 3);
    private JoystickButton wristUpButton = new JoystickButton(Constants.SECOND, 5);
    private JoystickButton wristDownButton = new JoystickButton(Constants.SECOND, 6);
    private JoystickButton poseMidButton = new JoystickButton(Constants.SECOND, 7);
    private JoystickButton poseHighButton = new JoystickButton(Constants.SECOND, 8);
    private JoystickButton poseShelfButton = new JoystickButton(Constants.SECOND, 9);
    private JoystickButton poseConeUpButton = new JoystickButton(Constants.SECOND, 11);
    private JoystickButton poseIntakeButton = new JoystickButton(Constants.SECOND, 12);

    @Override
    public void robotInit() {
        //Start the RoboRIO kernal file system nonsense
        try {    
            ReflectingLogger.getMount("XDLOL");
        } catch (Exception e) {}
        initButtons();
        CommandScheduler.getInstance().enable();
        manager = new SubsystemManager(
            Arrays.asList(
                Manipulator.getInstance(),
                Arm.getInstance(),
                DriveTrain.getInstance(),
                Lights.getInstance()
            ),
            true
        );

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

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

    @Override
    public void robotPeriodic() {
        manager.outputTelemetry();  
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        enabledLooper.stop();

        StateMachine.getInstance().assertStop();

        disabledLooper.start();
    }

    @Override
    public void autonomousInit() {
        disabledLooper.stop();
        Arm.getInstance().setMode(ArmMode.CLOSED_LOOP);
        DriveTrain.getInstance().reset();
        enabledLooper.start();
        AutoChooser.getInstance().run_from_selection();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        disabledLooper.stop();

        // Reset anything here
        DriveTrain.getInstance().reset();
        enabledLooper.start();
        Arm.getInstance().setMode(ArmMode.CLOSED_LOOP);
        DriveTrain.getInstance().setFieldRel();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        Arm.getInstance().setMode(ArmMode.DISABLED);
        disabledLooper.stop();
        enabledLooper.start();
    }

    @Override
    public void testPeriodic() {}

    public void initButtons() {
        // driveGearButton.whileTrue(Action.toCommand(new GearChangeAction()));
        intakeReverseButton.whileTrue(Action.toCommand(new RunIntakeAction(Constants.ANYTHING_OUT_POWER)));
        intakeSpitButton.whileTrue(Action.toCommand(new RunIntakeAction(Constants.ANYTHING_OUT_POWER)));
        intakeButton.whileTrue(Action.toCommand(new RunIntakeAction(Constants.INTAKE_POWER, false)));
        // autoLevelButton.whileTrue(Action.toCommand(new TeleopLevelAction()));
        // gyroLockButton.whileTrue(Action.toCommand(new GyroLockAction()));
        //limelightPipeButton.onTrue(Action.toCommand(new SetPipelineAction()));
        //unStowButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.UNSTOW)));
        zeroPoseButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.ZERO)));
        // revDriveTrainButton.onTrue(Action.toCommand(new ReverseDriveAction()));

        toggleDriveModeButton.onTrue(Action.toCommand(new DriveSwitchRobotMode()));
        resetGyroButton.onTrue(Action.toCommand(new DriveZeroGyro()));

        slideButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.SLIDE)));
        poseMidButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.MID)));
        poseHighButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.HIGH)));
        poseShelfButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.SHELF)));
        poseConeUpButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.CONE_UP)));
        poseIntakeButton.onTrue(Action.toCommand(new ArmPoseAction(ArmPose.INTAKE)));

        wristUpButton.whileTrue(Action.toCommand(new MoveWristAction(-.33)));
        wristDownButton.whileTrue(Action.toCommand(new MoveWristAction(.33)));
        cycleButton.whileTrue(Action.toCommand(new CycleArmAction()));
    }
}

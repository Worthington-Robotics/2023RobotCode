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
import frc.lib.statemachine.Action;
import frc.robot.actions.drive.DriveLevelAction;
import frc.robot.actions.drive.TeleopLevelAction;
import frc.robot.actions.drive.DriveTurnActionLimelight;
import frc.robot.actions.drive.GearChangeAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.manipulator.RunPivotAction;
import frc.robot.actions.drive.GyroLockAction;
import frc.robot.actions.drive.SetPositionAction;

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
    private JoystickButton transmissionButton = new JoystickButton(Constants.MASTER, 1);
    private JoystickButton limelightRotateButton = new JoystickButton(Constants.MASTER, 9);
    private JoystickButton autoLevelButton = new JoystickButton(Constants.MASTER, 7);
    private JoystickButton intakeButton = new JoystickButton(Constants.MASTER, 3);
    private JoystickButton intakeReverseButton = new JoystickButton(Constants.MASTER, 4);
    private JoystickButton pivotIntakeButton = new JoystickButton(Constants.MASTER, 2);
    private JoystickButton gryoLockButton = new JoystickButton(Constants.MASTER, 4);
    private JoystickButton resetPoseButton = new JoystickButton(Constants.MASTER, 8);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        manager = new SubsystemManager(
            Arrays.asList(
                PoseEstimator.getInstance(),
                Arm.getInstance(),
                DriveTrain.getInstance(),
                Lights.getInstance()
            ),
            true
        );

        // Create the master looper threads
        DriveTrajectoryGenerator.getInstance();
        enabledLooper = new Looper();
        disabledLooper = new Looper();

        // Register the looper threads to the manager to use for enabled and disabled
        manager.registerEnabledLoops(enabledLooper);
        manager.registerDisabledLoops(disabledLooper);

        // Add any additional logging sources for capture
        manager.addLoggingSource(Arrays.asList(StateMachine.getInstance()));

        initButtons();
        CommandScheduler.getInstance().enable();
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

        StateMachine.getInstance().assertStop();
        DriveTrain.getInstance().reset();
        PoseEstimator.getInstance().reset();
        Manipulator.getInstance().reset();

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

        // Reset anything here
        DriveTrain.getInstance().reset();
        enabledLooper.start();
        Lights.getInstance().reset();

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
        initButtons();
        Lights.getInstance().reset();
        DriveTrain.getInstance().reset();
        DriveTrain.getInstance().setOpenLoop();
        PoseEstimator.getInstance().reset();
        Manipulator.getInstance().reset();

        enabledLooper.start();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        disabledLooper.stop();

        // Reset anything here
        Dummy.getInstance().reset();
        DriveTrain.getInstance().reset();
        Manipulator.getInstance().reset();
        enabledLooper.start();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {}

    public void initButtons() {
        transmissionButton.whileTrue(Action.toCommand(new GearChangeAction()));
        intakeReverseButton.whileTrue(Action.toCommand(new RunIntakeAction(Constants.ANYTHING_OUT_POWER)));
        intakeButton.whileTrue(Action.toCommand(new RunIntakeAction(Constants.INTAKE_POWER)));
        pivotIntakeButton.whileTrue(Action.toCommand(new RunPivotAction()));
        autoLevelButton.whileTrue(Action.toCommand(new TeleopLevelAction()));
        limelightRotateButton.whileTrue(Action.toCommand(new DriveTurnActionLimelight()));
        resetPoseButton.onTrue(Action.toCommand(new SetPositionAction(0, 0, 0)));
        gryoLockButton.whileTrue(Action.toCommand(new GyroLockAction()));
    }
}

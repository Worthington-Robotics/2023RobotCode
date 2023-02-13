/*----------------------------------------------------------------------------*/
/* Copyright (c) 1892-1893 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.loops.Looper;
import frc.lib.models.DriveTrajectoryGenerator;
import frc.lib.statemachine.StateMachine;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.IntakePosition;
import frc.lib.statemachine.Action;
import frc.robot.actions.RunIntakeAction;
import frc.robot.actions.SuperstructureActions;

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
    private JoystickButton intakeCubeButton = new JoystickButton(Constants.MASTER, 2);
    private JoystickButton intakeConeButton = new JoystickButton(Constants.MASTER, 4);
    private JoystickButton intakeReverseButton = new JoystickButton(Constants.MASTER, 3);
    private JoystickButton intakeDownButton = new JoystickButton(Constants.MASTER, 10);
    private JoystickButton intakeUpButton = new JoystickButton(Constants.MASTER, 9);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        manager = new SubsystemManager(
            Arrays.asList(
                SuperStructure.getInstance(),
                Arm.getInstance(),
                DriveTrain.getInstance()
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
    }

    @Override
    public void disabledInit() {
        enabledLooper.stop();

        StateMachine.getInstance().assertStop();

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
        enabledLooper.start();
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
        SuperStructure.getInstance().reset();
        Dummy.getInstance().reset();
        DriveTrain.getInstance().reset();

        enabledLooper.start();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {}

    public void initButtons() {
        intakeConeButton.whileTrue(Action.toCommand(new RunIntakeAction(Constants.CONE_IN_POWER)));
        intakeReverseButton.whileTrue(Action.toCommand(new RunIntakeAction(Constants.ANYTHING_OUT_POWER)));
        intakeCubeButton.whileTrue(Action.toCommand(new RunIntakeAction(Constants.CUBE_IN_POWER)));
        intakeUpButton.onTrue(Action.toCommand(new SuperstructureActions.MoveIntakeAction(IntakePosition.kUp)));
        intakeDownButton.onTrue(Action.toCommand(new SuperstructureActions.MoveIntakeAction(IntakePosition.kDown)));
    }
}

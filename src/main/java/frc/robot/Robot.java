/*----------------------------------------------------------------------------*/
/* Copyright (c) 1892-1893 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.loops.Looper;
import frc.lib.pathplanner.PPStateMachine;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Manipulator;
import frc.robot.subsystems.arm.Arm.ArmMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private SubsystemManager manager;
    private JoystickButtonManager buttonManager = new JoystickButtonManager();
    private Looper enabledLooper, disabledLooper;

    @Override
    public void robotInit() {
        // Start the RoboRIO kernal file system nonsense
        CommandScheduler.getInstance().enable();
        // CameraServer.startAutomaticCapture();
        manager = new SubsystemManager(
                Arrays.asList(
                        SwerveDrive.getInstance(),
                        Lights.getInstance(),
                        Manipulator.getInstance(),
                        Arm.getInstance()
                        ),
                true);

        DataLogManager.logNetworkTables(true);
        DataLogManager.start(null, null, 0.02);
        // DriverStation.startDataLog(DataLogManager.getLog());

        enabledLooper = new Looper();
        disabledLooper = new Looper();

        // Register the looper threads to the manager to use for enabled and disabled
        manager.registerEnabledLoops(enabledLooper);
        manager.registerDisabledLoops(disabledLooper);

        AutoChooser.getInstance().getAutos();
        AutoChooser.getInstance().logAuto();
        AutoChooser.getInstance().printAutos();
    }

    @Override
    public void robotPeriodic() {
        manager.outputTelemetry();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        enabledLooper.stop();
        PPStateMachine.getInstance().assertStop();
        Lights.getInstance().setState(Lights.State.INIT);
        disabledLooper.start();
    }

    @Override
    public void autonomousInit() {
        disabledLooper.stop();
        Arm.getInstance().setMode(ArmMode.CLOSED_LOOP);
        Lights.getInstance().setState(Lights.State.AUTO);
        Manipulator.getInstance().setAuto(true);
        AutoChooser.getInstance().runFromSelection();
        enabledLooper.start();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        disabledLooper.stop();
        // Reset anything here
        Arm.getInstance().setMode(ArmMode.CLOSED_LOOP);
        Manipulator.getInstance().setAuto(false);
        SwerveDrive.getInstance().setState(SwerveDrive.State.FieldRel);
        PPStateMachine.getInstance().clearTrajectory();
        Lights.getInstance().setState(Lights.State.TELEOP);
        enabledLooper.start();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        disabledLooper.stop();
        Arm.getInstance().setMode(ArmMode.DISABLED);
        enabledLooper.start();
    }

    @Override
    public void testPeriodic() {
    }
}

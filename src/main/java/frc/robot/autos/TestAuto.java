package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.vision.AutoPipelineAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.wait.TurretWaitAction;
import frc.robot.subsystems.Arm.ArmPose;
import frc.robot.subsystems.VisionLink.LimelightPipeline;

public class TestAuto extends StateMachineDescriptor {
    public TestAuto() {
<<<<<<< HEAD
        addSequential(new MoveForwardAction(-25 * Constants.ENCODER_PER_INCH, 0), 6000); //move forward to get to the target
        addSequential(new ArmPoseAction(ArmPose.FIRST_MOVE), 200);
        addSequential(new PoseWaitAction(), 2000);
        addSequential(new ArmPoseAction(ArmPose.CONE_MID_FRONT), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 2000);
        addSequential(new TimeWaitAction(), 500);

        addSequential(new MoveForwardAction(25 * Constants.ENCODER_PER_INCH, 0), 6000); //move forward to get to the target
        addSequential(new RotateTurretAction(-(15) * Constants.TURRET_TPD), 200); //rotate turret 15 degrees right 6
        addSequential(new TurretWaitAction(), 2000); // 7
        addSequential(new AutoPipelineAction(LimelightPipeline.Low), 200);
        addSequential(new LLHoldAction(true, true), 4000); //use limelight for correction 9
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500); //release cone

        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 200);
        addSequential(new PoseWaitAction(), 2000);
        addSequential(new RotateTurretAction(167 * Constants.TURRET_TPD), 200);
        addSequential(new TimeWaitAction(), 1400);

        Action[] rotateMoveToCube = {new MoveForwardAction(-120 * Constants.ENCODER_PER_INCH, 0)};
        addParallel(rotateMoveToCube, 7000);
        // addSequential(new RotateTurretAction(150 * Constants.TURRET_ENCODER_PER_DEGREE), 8000); 
        // addSequential(new TurretWaitAction(), 7000);

        addSequential(new ArmPoseAction(ArmPose.INTAKE), 200);
        addSequential(new PoseWaitAction(), 7000);
        addSequential(new TimeWaitAction(), 500);

        
        Action[] runIntake = {new MoveForwardAction(-50 * Constants.ENCODER_PER_INCH, 0), new RunIntakeAction(Constants.INTAKE_POWER)};
=======
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 3000);
        addSequential(new PoseWaitAction(), 7000);
        addSequential(new ArmPoseAction(ArmPose.CONE_MID),3000); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 7000);

        addSequential(new MoveForwardAction(20 * Constants.ENCODER_PER_INCH, 0), 5000); //move forward to get to the target
        addSequential(new RotateTurretAction(-(15) * Constants.TURRET_ENCODER_PER_DEGREE), 3000); //rotate turret 15 degrees right
        addSequential(new SetPipelineAction(LimelightPipeline.Low), 4000);
        addSequential(new TurretWaitAction(), 7000);
        addSequential(new LLHoldAction(), 4000); //use limelight for correction
        addSequential(new TimeWaitAction(), 2000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500); //release cone
        addSequential(new RotateTurretAction(0), 8000); //rotate turret back to center
        addSequential(new TurretWaitAction(), 7000);

        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 3000);
        addSequential(new PoseWaitAction(), 7000);

        Action[] rotateMoveToCube = {new MoveForwardAction(-149 * Constants.ENCODER_PER_INCH, 0), new RotateTurretAction(90 * Constants.TURRET_ENCODER_PER_DEGREE)};
        addParallel(rotateMoveToCube, 5000);
        addSequential(new TurretWaitAction(), 7000);
        addSequential(new RotateTurretAction(150 * Constants.TURRET_ENCODER_PER_DEGREE), 8000); 
        addSequential(new TurretWaitAction(), 7000);

        addSequential(new ArmPoseAction(ArmPose.INTAKE), 3000);
        addSequential(new PoseWaitAction(), 7000);
        Action[] runIntake = {new MoveForwardAction(-6 * Constants.ENCODER_PER_INCH, 0), new RunIntakeAction(Constants.INTAKE_POWER)};
>>>>>>> c2aec83e56e319ffe7203bd744b1e0a85f51523f
        addParallel(runIntake, 1000);
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);
        addSequential(new PoseWaitAction(), 7000);

<<<<<<< HEAD
        Action[] rotateMoveToCubeTarget = {new MoveForwardAction(148 * Constants.ENCODER_PER_INCH, 0), new RotateTurretAction(15 * Constants.TURRET_TPD)};
        addParallel(rotateMoveToCubeTarget, 8000);
        addSequential(new ArmPoseAction(ArmPose.CUBE_MID_FRONT),3000); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 2000);
        addSequential(new TimeWaitAction(), 1000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500);
        
=======
        // Action[] rotateMoveToCubeTarget = {new MoveForwardAction(129 * Constants.ENCODER_PER_INCH, 0), new RotateTurretAction(90 * Constants.TURRET_ENCODER_PER_DEGREE)};
        // addParallel(rotateMoveToCubeTarget, 8000);
        // addSequential(new ArmPoseAction(ArmPose.CONE_MID),3000); //put the robot into the set pose
        // addSequential(new PoseWaitAction(), 7000);

>>>>>>> c2aec83e56e319ffe7203bd744b1e0a85f51523f
        // addSequential(new MoveForwardAction(20 * Constants.ENCODER_PER_INCH, 0), 5000); //move forward to get to the target
        // addSequential(new RotateTurretAction(15 * Constants.TURRET_ENCODER_PER_DEGREE), 3000); //rotate turret 15 degrees right
        // addSequential(new TurretWaitAction(), 7000); 
        // addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500); //release cone
        // addSequential(new RotateTurretAction(0), 8000); //rotate turret back to center
        // addSequential(new TurretWaitAction(), 7000);

        // Action[] rotateMoveToCone = {new MoveForwardAction(-120 * Constants.ENCODER_PER_INCH, 0), new RotateTurretAction(90 * Constants.TURRET_ENCODER_PER_DEGREE)};
        // addParallel(rotateMoveToCone, 5000);
        // addSequential(new TurretWaitAction(), 7000);
        // addSequential(new RotateTurretAction(178* Constants.TURRET_ENCODER_PER_DEGREE), 8000); 
        // addSequential(new TurretWaitAction(), 7000);
        // addSequential(new DriveTurnAction(-45), 5000);

        // addSequential(new ArmPoseAction(ArmPose.INTAKE), 3000);
        // addSequential(new PoseWaitAction(), 7000);
        // Action[] runIntakeTwo = {new MoveForwardAction(-6 * Constants.ENCODER_PER_INCH, 0), new RunIntakeAction(Constants.INTAKE_POWER)};
        // addParallel(runIntakeTwo, 1000);
        // addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);
        // addSequential(new PoseWaitAction(), 7000);

    }

}
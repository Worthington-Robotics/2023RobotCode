package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.vision.AutoPipelineAction;
import frc.robot.subsystems.Arm.ArmPose;
import frc.robot.subsystems.VisionLink.LimelightPipeline;

public class TestAuto extends StateMachineDescriptor {
    public TestAuto() {
        addSequential(new ArmPoseAction(ArmPose.FIRST_MOVE), 200);
        addSequential(new PoseWaitAction(), 300);
        addSequential(new ArmPoseAction(ArmPose.CONE_MID_FRONT), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
        addParallel(new Action[] {new MoveForwardAction(25 * Constants.ENCODER_PER_INCH, 0), new RotateTurretAction(-(15) * Constants.TURRET_TPD), new AutoPipelineAction(LimelightPipeline.Low)}, 6000); //move forward to get to the target
        addSequential(new LLHoldAction(true, true), 3000); //use limelight for correction 9
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250); //release cone
        addParallel(new Action[] {new ArmPoseAction(ArmPose.INTAKE_LITE), new RotateTurretAction(182 * Constants.TURRET_TPD)}, 3000);
        //END CONE
        addParallel(new Action[] {new MoveForwardAction(-150 * Constants.ENCODER_PER_INCH, 0)}, 8000);
        addParallel(new Action[] {new MoveForwardAction(-30 * Constants.ENCODER_PER_INCH, 0), new ArmPoseAction(ArmPose.INTAKE), new RunIntakeAction(Constants.INTAKE_POWER)}, 3000);
        addParallel(new Action[] {new MoveForwardAction(166 * Constants.ENCODER_PER_INCH, 0), new RotateTurretAction(7 * Constants.TURRET_TPD), new ArmPoseAction(ArmPose.TRANSIT)}, 6000);

        addSequential(new ArmPoseAction(ArmPose.CUBE_MID_FRONT),50); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1500);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        
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
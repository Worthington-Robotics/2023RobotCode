package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.subsystems.Arm.ArmPose;
public class TestAuto extends StateMachineDescriptor {
    public TestAuto() {
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 20);
        addSequential(new PoseWaitAction(), 4000);
        //addSequential(new TimeWaitAction(), 3000);
        addSequential(new ArmPoseAction(ArmPose.CONE_HIGH), 8000); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 4000);
        addSequential(new MoveForwardAction(15 * Constants.ENCODER_PER_INCH, 0), 5000); //move forward 1 foot to get to the target
        addSequential(new RotateTurretAction(-(15) * Constants.TURRET_ENCODER_PER_DEGREE), 3000); //rotate turret 15 degrees right
        addSequential(new PoseWaitAction(), 4000);
        addSequential(new LLHoldAction(), 3000); //use limelight for correction
        addSequential(new TimeWaitAction(), 4000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500); //release cone
        addSequential(new RotateTurretAction(0), 8000); //rotate turret back to center
        addSequential(new TimeWaitAction(), 3000);
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 3000);
        addSequential(new PoseWaitAction(), 4000);
        addSequential(new MoveForwardAction(-15 * Constants.ENCODER_PER_INCH, 0), 5000); //back up 1 foot
        //addSequential(new TimeWaitAction(), 4000);
        //addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);
    }
}
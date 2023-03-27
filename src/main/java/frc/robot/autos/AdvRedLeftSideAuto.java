package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.drive.NonblockingMoveAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.ReachLineWaitAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.vision.AutoPipelineAction;
import frc.robot.actions.vision.SetPipelineAction;
import frc.robot.actions.vision.SnapshotAction;
import frc.robot.subsystems.Arm.ArmPose;

public class AdvRedLeftSideAuto extends StateMachineDescriptor {
    public AdvRedLeftSideAuto() {
        addSequential(new ArmPoseAction(ArmPose.SLIDE), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new RotateTurretAction(-180 * Constants.TURRET_TPD, false), 4000);
        addSequential(new ArmPoseAction(ArmPose.MID), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new ArmPoseAction(ArmPose.HIGH), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new LLHoldAction(true, true), 1000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new ArmPoseAction(ArmPose.SLIDE), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
    
        addSequential(new RotateTurretAction(-20 * Constants.TURRET_TPD, false), 4000);
        //END CONE
        addParallel(new Action[] {new NonblockingMoveAction(170 * Constants.ENCODER_PER_INCH, 0), new ReachLineWaitAction(true, 120 * Constants.ENCODER_PER_INCH), new ArmPoseAction(ArmPose.UNSTOW)}, 8000);
        addParallel(new Action[] {new ArmPoseAction(ArmPose.INTAKE), new RunIntakeAction(Constants.INTAKE_POWER)}, 5000);
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);
        addSequential(new PoseWaitAction(), 1000);
        addParallel(new Action[] {new NonblockingMoveAction(-167 * Constants.ENCODER_PER_INCH, 0), new ArmPoseAction(ArmPose.TRANSIT), new ReachLineWaitAction(false, -150 * Constants.ENCODER_PER_INCH)}, 13000);

        addSequential(new RotateTurretAction(-170 * Constants.TURRET_TPD, false),3000);

        addSequential(new ArmPoseAction(ArmPose.MID),250); //put the robot into the set pose
       addSequential(new PoseWaitAction(), 4000);
        addSequential(new TimeWaitAction(), 250);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
    }

}

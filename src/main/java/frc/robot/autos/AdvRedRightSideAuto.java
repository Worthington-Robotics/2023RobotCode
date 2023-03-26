package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.lib.statemachine.Action;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.drive.NonblockingMoveAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.ReachLineWaitAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.subsystems.Arm.ArmPose;

public class AdvRedRightSideAuto extends StateMachineDescriptor {
    public AdvRedRightSideAuto() {
        addSequential(new ArmPoseAction(ArmPose.MID), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new RotateTurretAction(180 * Constants.TURRET_TPD, false), 4000);
        addSequential(new ArmPoseAction(ArmPose.HIGH), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new LLHoldAction(true, true), 1000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new ArmPoseAction(ArmPose.MID), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
    
        addSequential(new RotateTurretAction(20 * Constants.TURRET_TPD, false), 4000);
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 200);
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new TimeWaitAction(), 250);

        addParallel(new Action[] {new NonblockingMoveAction(160 * Constants.ENCODER_PER_INCH, 0), new ReachLineWaitAction(true, 40 * Constants.ENCODER_PER_INCH)}, 8000);
        addSequential(new ArmPoseAction(ArmPose.INTAKE), 200);
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new RunIntakeAction(Constants.INTAKE_POWER), 9000);
        // addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);
        // addSequential(new PoseWaitAction(), 3000);
        // addParallel(new Action[] {new NonblockingMoveAction(-167 * Constants.ENCODER_PER_INCH, 0), new ArmPoseAction(ArmPose.TRANSIT), new ReachLineWaitAction(false, -120 * Constants.ENCODER_PER_INCH)}, 13000);

        // addSequential(new ArmPoseAction(ArmPose.MID),50); //put the robot into the set pose
        // addSequential(new PoseWaitAction(), 3000);
       // addSequential(new RotateTurretAction(173 * Constants.TURRET_TPD, false), 4000);
        // addSequential(new TimeWaitAction(), 250);
        // addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
    }
}

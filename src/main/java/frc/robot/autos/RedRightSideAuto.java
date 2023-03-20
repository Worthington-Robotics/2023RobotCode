package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.lib.statemachine.Action;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.arm.TurretHoldAction;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.drive.UnblockingMoveAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.ReachLineWaitAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.vision.AutoPipelineAction;
import frc.robot.actions.vision.SnapshotAction;
import frc.robot.subsystems.Arm.ArmPose;

public class RedRightSideAuto extends StateMachineDescriptor {
    public RedRightSideAuto() {
        addSequential(new ArmPoseAction(ArmPose.FIRST_MOVE), 200);
        addSequential(new PoseWaitAction(), 300);
        addSequential(new ArmPoseAction(ArmPose.CONE_MID_FRONT), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
        addParallel(new Action[] {new MoveForwardAction(28.5 * Constants.ENCODER_PER_INCH, 0), new RotateTurretAction(15 * Constants.TURRET_TPD, false), new AutoPipelineAction(0)}, 4000); //move forward to get to the target
        addParallel(new Action[] {new LLHoldAction(true, true), new SnapshotAction()}, 3000); //use limelight for correction 9
        addParallel(new Action[] {new RunIntakeAction(Constants.ANYTHING_OUT_POWER), new SnapshotAction()}, 250);
        addParallel(new Action[] {new ArmPoseAction(ArmPose.INTAKE_LITE), new RotateTurretAction(-182 * Constants.TURRET_TPD, false)}, 4000);
        addSequential(new TimeWaitAction(), 250);
        //END CONE
        addParallel(new Action[] {new UnblockingMoveAction(-170 * Constants.ENCODER_PER_INCH, 0), new TurretHoldAction(-182 * Constants.TURRET_TPD), new ReachLineWaitAction(false, -120 * Constants.ENCODER_PER_INCH)}, 8000);
        addParallel(new Action[] {new ArmPoseAction(ArmPose.INTAKE), new RunIntakeAction(Constants.INTAKE_POWER)}, 9000);
        addParallel(new Action[] {new UnblockingMoveAction(167 * Constants.ENCODER_PER_INCH, 0), new ArmPoseAction(ArmPose.INTAKE_LITE), new ReachLineWaitAction(true, 120 * Constants.ENCODER_PER_INCH)}, 13000);

        addSequential(new ArmPoseAction(ArmPose.CUBE_MID_FRONT),50); //put the robot into the set pose
        addParallel(new Action[] {new RotateTurretAction(-7 * Constants.TURRET_TPD, false), new PoseWaitAction()}, 1750);
        addSequential(new TimeWaitAction(), 250);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
    }
}

package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.PinToggleAction;
import frc.lib.statemachine.Action;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.drive.GearChangeAction;
import frc.robot.actions.drive.NonblockingMoveAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.ArmPose;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.wait.PoseWaitAction;

public class PitTestAuto extends StateMachineDescriptor {
    // TODO: Adjust these values so its not 10 seconds between each pose
    int patience = 200;
    int pose_patience = 5000;
    int intake_patience = 5000;
    int drive_patience = 2000;

    public PitTestAuto() {
        // Test Arm Poses
        // addSequential(new ArmPoseAction(ArmPose.FIRST_MOVE), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new ArmPoseAction(ArmPose.TRANSIT), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new ArmPoseAction(ArmPose.UNSTOW), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);


        // addSequential(new ArmPoseAction(ArmPose.INTAKE), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new RunIntakeAction(1), drive_patience);


        // addSequential(new ArmPoseAction(ArmPose.CUBE_MID), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new ArmPoseAction(ArmPose.CUBE_MID_FRONT), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new ArmPoseAction(ArmPose.UNSTOW), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new ArmPoseAction(ArmPose.CUBE_HIGH), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new RunIntakeAction(-1), patience);

        // addSequential(new ArmPoseAction(ArmPose.CONE_MID), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new ArmPoseAction(ArmPose.CONE_MID_FRONT), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new ArmPoseAction(ArmPose.UNSTOW), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new ArmPoseAction(ArmPose.CONE_HIGH), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new ArmPoseAction(ArmPose.FIRST_MOVE), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new ArmPoseAction(ArmPose.STOWN), this.patience);
        // addSequential(new PoseWaitAction(), this.pose_patience);
        // addSequential(new TimeWaitAction(), pose_patience);
        // addSequential(new PinToggleAction(), this.pose_patience);

        /* 
        // Intake Cube (Consider Intaking During Easier Pose)
        addSequential(new ArmPoseAction(ArmPose.INTAKE_LITE), this.patience); // Move to cube intake position
        addSequential(new PoseWaitAction(), this.pose_patience);
        addSequential(new RunIntakeAction(Constants.INTAKE_POWER), this.patience); // intake cube
        addSequential(new TimeWaitAction(), this.intake_patience);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), this.patience); // Spit out cube
        
        // Intake Cone
        addSequential(new ArmPoseAction(ArmPose.INTAKE_LITE), this.patience); // Move to cone intake position
        addSequential(new PoseWaitAction(), this.pose_patience);
        addSequential(new RunIntakeAction(Constants.INTAKE_POWER), this.patience); // intake cone
        addSequential(new TimeWaitAction(), this.intake_patience);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), this.patience); // Spit out cone

        // Test Turret

        // Test Drivetrain (add waits)
        addSequential(new UnblockingMoveAction(100 * Constants.ENCODER_PER_INCH, 0), this.drive_patience);
        addSequential(new TimeWaitAction(), this.drive_patience);
        //addSequential(new DriveLevelAction(0), this.drive_patience);
        addSequential(new TimeWaitAction(), this.drive_patience);
        addSequential(new DriveTurnAction(90), this.drive_patience);
        addSequential(new TimeWaitAction(), this.drive_patience);
        addParallel(new Action[] {new GearChangeAction(),new UnblockingMoveAction(100 * Constants.ENCODER_PER_INCH, 90)}, this.drive_patience);
        // UnblockMoveAction, TeleopLevelAction, Gyrolock, Limelight, Autolevel
        
        // Stow and Pin
        addSequential(new ArmPoseAction(ArmPose.STOWN), this.patience);
        addSequential(new PoseWaitAction(), this.pose_patience);
        // Switch to teleop and pin
        */
    }
}

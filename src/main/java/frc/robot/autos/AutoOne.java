package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldPipelineAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.subsystems.Arm.ArmPose;
import frc.robot.subsystems.VisionLink.LimelightPipeline;
import frc.robot.Constants;


public class AutoOne extends StateMachineDescriptor {
    //TURRET_ENCODER_PER_DEGREE = 227.56
    //ENCODER_PER_INCH = 3904.5
    public AutoOne() {
        //robot faces the target
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 3000);
        addSequential(new ArmPoseAction(ArmPose.CONE_HIGH), 8000); //put the robot into the set pose
        addSequential(new MoveForwardAction(46854, 0), 6000); //move forward 1 foot to get to the target
        addSequential(new RotateTurretAction(-(15) * Constants.TURRET_ENCODER_PER_DEGREE), 3000); //rotate turret 15 degrees right
        addSequential(new LLHoldPipelineAction(LimelightPipeline.High), 3000); //use limelight for correction
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500); //release cone
        addSequential(new RotateTurretAction(90 * Constants.TURRET_ENCODER_PER_DEGREE), 3000); //rotate turret back to center
        addSequential(new MoveForwardAction(-46854, 0), 2000); //back up 1 foot
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);

        Action[] rotateMoveToCube = {new MoveForwardAction(-265000, 0), new RotateTurretAction(180 * Constants.TURRET_ENCODER_PER_DEGREE)}; // rotate 178 counterclockwise
        addParallel(rotateMoveToCube, 5000);

        addSequential(new ArmPoseAction(ArmPose.INTAKE), 3000);
        Action[] runIntake = {new MoveForwardAction(-10000, 0), new RunIntakeAction(Constants.INTAKE_POWER)};
        addParallel(runIntake, 4000);
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);

        Action[] rotateMoveToTarget = {new MoveForwardAction(275000, 0), new RotateTurretAction((15) * Constants.TURRET_ENCODER_PER_DEGREE)};
        addParallel(rotateMoveToTarget , 5000); //move back to goal

        addSequential((new ArmPoseAction(ArmPose.CUBE_HIGH)), 3000); //put the robot into the set pose
        addSequential(new MoveForwardAction(46854, 0), 2000); //move forward to get to the target
        
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500); //spit out the cube
        addSequential(new MoveForwardAction(-46854, 0), 2000); //back up slightly
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);
        
        Action[] rotateMoveToConeTwo = {new MoveForwardAction(-200000, 0), new RotateTurretAction(40505.68)};
        addParallel(rotateMoveToConeTwo, 5000); //move backwards partway
        addSequential(new DriveTurnAction(-45), 3000);
        addSequential(new MoveForwardAction(-90000, -45), 5000); //move towards cone diagonally

        addSequential(new ArmPoseAction(ArmPose.INTAKE), 3000);
        addParallel(runIntake, 4000);
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);

        addSequential(new MoveForwardAction(100000, -45), 5000); //move diagonally back
        addSequential(new DriveTurnAction(0), 4000); //rotate drivetrain to face target
        Action[] rotateMoveToSecondTarg = {new MoveForwardAction(200000, 0), new RotateTurretAction(30 * Constants.TURRET_ENCODER_PER_DEGREE)};
        addParallel(rotateMoveToSecondTarg , 5000); //move back to community area

        addSequential((new ArmPoseAction(ArmPose.CONE_HIGH)), 3000); //put the robot into the set pose
    

    
    }
}

package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.arm.LLHoldPipelineAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.arm.TurretHoldAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.time.DriveWaitAction;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPose;
import frc.robot.subsystems.VisionLink.LimelightPipeline;
import frc.robot.Constants;

public class AutoThree extends StateMachineDescriptor {
     //TURRET_ENCODER_PER_DEGREE = 227.56
    //ENCODER_PER_INCH = 3904.5
    public AutoThree() {
        //robot faces the target
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 3000);
        addSequential(new ArmPoseAction(ArmPose.CONE_HIGH), 3000); //put the robot into the set pose
        addSequential(new MoveForwardAction(46854, 0), 2000); //move forward 1 foot to get to the target
        addSequential(new RotateTurretAction(3413), 3000); //rotate turret 15 degrees left
        addSequential(new LLHoldPipelineAction(LimelightPipeline.High), 2000); //use limelight for correction
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500); //release cone
        addSequential(new RotateTurretAction(0), 3000); //rotate turret back to center
        addSequential(new MoveForwardAction(-46854, 0), 2000); //back up 1 foot
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);

        Action[] rotateMoveToCube = {new MoveForwardAction(-275000, 0), new RotateTurretAction(-40505.68)}; // rotate 178 counterclockwise
        addParallel(rotateMoveToCube, 5000);

        addSequential(new ArmPoseAction(ArmPose.INTAKE), 3000);
        addSequential(new RunIntakeAction(Constants.INTAKE_POWER), 1000);
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);

        Action[] rotateMoveToTarget = {new MoveForwardAction(275000, 0), new RotateTurretAction(-3413)};
        addParallel(rotateMoveToTarget , 5000); //move back to goal

        addSequential((new ArmPoseAction(ArmPose.CUBE_HIGH)), 3000); //put the robot into the set pose
        addSequential(new MoveForwardAction(46854, 0), 2000); //move forward to get to the target
        
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500); //spit out the cube
        addSequential(new MoveForwardAction(-46854, 0), 2000); //back up slightly
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);
        addSequential(new RotateTurretAction(0), 3000); //rotate turret back to center
        
        Action[] rotateMoveToConeTwo = {new MoveForwardAction(-200000, 0), new RotateTurretAction(-40505.68)};
        addParallel(rotateMoveToConeTwo, 5000); //move backwards partway
        addSequential(new DriveTurnAction(45), 3000);
        addSequential(new MoveForwardAction(-100000, 45), 5000); //move towards cone diagonally

        addSequential(new ArmPoseAction(ArmPose.INTAKE), 3000);
        addSequential(new RunIntakeAction(Constants.INTAKE_POWER), 1000);
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);

        addSequential(new MoveForwardAction(100000, 45), 5000); //move diagonally
        addSequential(new DriveTurnAction(0), 4000); //rotate drivetrain to face target
        Action[] rotateMoveToSecondTarg = {new MoveForwardAction(200000, 0), new RotateTurretAction(-6826.8)};
        addParallel(rotateMoveToSecondTarg , 5000); //move back to community area

        addSequential((new ArmPoseAction(ArmPose.CONE_HIGH)), 3000); //put the robot into the set pose
    

    
    }

}

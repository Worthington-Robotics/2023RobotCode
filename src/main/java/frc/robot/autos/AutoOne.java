package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.drive.WaitAction;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LimelightTurretCorrectionAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.arm.TurretHoldAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPose;
import frc.robot.Constants;


public class AutoOne extends StateMachineDescriptor {
    //TURRET_ENCODER_PER_DEGREE = 227.56;
    public AutoOne() {
        //robot faces the target
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 3000);
        addSequential((new ArmPoseAction(ArmPose.CUBE_HIGH)), 3000); //put the robot into the set pose
        addSequential((new RotateTurretAction(4370)), 3413); //rotate turret 15 degrees left
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500); //spit out the cube
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);

        Action[] rotateMoveToCone = {new MoveForwardAction(-325000, 0), new RotateTurretAction(40505.68)}; // rotate 178 counterclockwise
        addParallel(rotateMoveToCone, 5000);

        addSequential(new ArmPoseAction(ArmPose.INTAKE), 3000);
        addSequential(new RunIntakeAction(Constants.INTAKE_POWER), 1000);
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);

        Action[] rotateMoveToTarget = {new MoveForwardAction(325000, 0), new RotateTurretAction(-5689)};
        addParallel(rotateMoveToTarget , 5000); //move back to goal

        addSequential((new ArmPoseAction(ArmPose.CONE_HIGH)), 3000); //put the robot into the set pose
        addSequential(new LimelightTurretCorrectionAction(), 2000); //use limelight for correction
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500); //spit out the cone 
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);
        
        Action[] rotateMoveToCube = {new MoveForwardAction(-211875, 0), new RotateTurretAction(40505.68)};
        addParallel(rotateMoveToCube, 5000); //move backwards partway
        addSequential(new DriveTurnAction(-45), 3000);
        addSequential(new MoveForwardAction(-115060, -45), 5000); //move towards cone

        addSequential(new ArmPoseAction(ArmPose.INTAKE), 3000);
        addSequential(new RunIntakeAction(Constants.INTAKE_POWER), 1000);
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);

        addSequential(new MoveForwardAction(115060, -45), 5000); //move back partway
        Action[] rotateMoveToSecondTarg = {new MoveForwardAction(211875, 0), new RotateTurretAction(6826.8)};
        addParallel(rotateMoveToSecondTarg , 5000); //move back to goal

        addSequential((new ArmPoseAction(ArmPose.CONE_HIGH)), 3000); //put the robot into the set pose
        addSequential(new LimelightTurretCorrectionAction(), 2000); //use limelight for correction
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500); //spit out the cone 
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);

        // addSequential(new MoveForwardAction(-320000, 0), 5000);
        // // addSequential(new RunIntakeAction(0.5), 5000);
        // addSequential(new WaitAction(), 1000);
        // //addSequential(new MoveForwardAction(15345, -90), 5000);
        // addSequential(new MoveForwardAction(-320000, 0), 5000);
        // addSequential(new WaitAction(), 2000);
    }
}

package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.lib.util.TimerBoolean;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Arm.ArmPose;

public class ChangePoseAction extends Action{

    ArmPose beginPose;
    ArmPose endPose;
    TimerBoolean firstPoseDuration = new TimerBoolean(3);

    public ChangePoseAction(ArmPose beginPose, ArmPose endPose) {
        this.beginPose = beginPose;
        this.endPose = endPose;
    }

    @Override
    public void onStart() {
        Arm.getInstance().setPose(beginPose);
        Manipulator.getInstance().resWrist();
        firstPoseDuration.start();
    }

    @Override
    public void onLoop() {
      if(firstPoseDuration.getBoolean()){
        Arm.getInstance().setPose(endPose);
      }
    }

    @Override
    public boolean isFinished() {
        return Arm.getInstance().getPose() == endPose;
    }

    @Override
    public void onStop() {   
    }

    
}

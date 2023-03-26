package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.lib.util.TimerBoolean;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Arm.ArmPose;

public class TECorrectionAction extends Action{

    double referenceDistance = 0;
    double extensionMultiplier = 0;

    double aprilTagDist;
    double deltaDist;
    double extensionChange;
    boolean search;
    boolean kill;
    boolean fire;
    TimerBoolean finished = new TimerBoolean(.25);
    double[] vals;

    public TECorrectionAction(boolean search) {
        vals = Arm.getInstance().getLLVals();
        this.search = search;
        fire = false;
        kill = false;
    }

    public TECorrectionAction(boolean search, boolean kill) {
        vals = Arm.getInstance().getLLVals();
        this.search = search;
        this.kill = kill;
        fire = false;
    }

    public TECorrectionAction(boolean search, boolean kill, boolean fire) {
        vals = Arm.getInstance().getLLVals();
        this.search = search;
        this.kill = kill;
        this.fire = fire;
    }

    
    @Override
    public void onStart() {
        aprilTagDist = Arm.aprilTagDistances[2];
        deltaDist = referenceDistance - aprilTagDist;
        extensionChange = deltaDist * extensionMultiplier;

        Arm.getInstance().correctExtension(extensionChange);

        if(Arm.getInstance().getPose() == ArmPose.HIGH){
            Arm.pipeline.setDouble(1);
        } else if (Arm.getInstance().getPose() == ArmPose.MID){
            Arm.pipeline.setDouble(0);
        }

    }

    @Override
    public void onLoop() {
        if((Arm.getInstance().getExtendEncoderError()) < Constants.EXTENSION_ENCODER_ERROR_ACCEPTANCE) { //go to turret correction after extension correction
            vals = Arm.getInstance().getLLVals();
            Arm.getInstance().turretHoldLock(true, Arm.getInstance().getTurretEncoder() -  (vals[0] * Constants.TURRET_TPD)); 
            if((kill ? (vals[1] > 0 && Math.abs(vals[0]) < 2) : false) 
            || !search && vals[1] < 1) {
                if(!finished.isStarted())
                    finished.start();
            } else {
                finished.stop();
                System.out.println("stopped /w " + vals[0] + " and " + vals[1] + " and " + kill);
            }
        } else {
            finished.reset(); //reset the timer if you haven't started turret correction
        }
    }

    @Override
    public boolean isFinished() {
        return finished.getBoolean();
    }

    @Override
    public void onStop() {
        if(fire && finished.getBoolean()) {
            Manipulator.getInstance().setIntakePower(-1);
        } 
        Arm.getInstance().turretHoldLock(false, 0);
        Arm.pipeline.setDouble(2);
    }
    
}

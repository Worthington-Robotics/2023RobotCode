package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.lib.util.TimerBoolean;

public class LLHoldAction extends Action{
        boolean search;
        boolean kill;
        TimerBoolean finished = new TimerBoolean(.3);
        double[] vals;

        public LLHoldAction(boolean search) {
            vals = Arm.getInstance().getLLVals();
            this.search = search;
            kill = false;
        }

        public LLHoldAction(boolean search, boolean kill) {
            vals = Arm.getInstance().getLLVals();
            this.search = search;
            this.kill = kill;
        }

        @Override
        public void onStart() {
            Arm.getInstance().turretHoldLock(true, Arm.getInstance().getTurretEncoder() -  (vals[0] * Constants.TURRET_TPD));
            
        }
    
        @Override
        public void onLoop() {
            vals = Arm.getInstance().getLLVals();
            Arm.getInstance().turretHoldLock(true, Arm.getInstance().getTurretEncoder() -  (vals[0] * Constants.TURRET_TPD)); 
            if((kill ? (vals[1] > 0 && Math.abs(vals[0]) < 1.5) : false) 
            || !search && vals[1] < 1) {
                finished.start();
            } else {
                finished.stop();
            }
        }
    
        @Override
        public boolean isFinished() {
            return finished.getBoolean();
        }
    
        @Override
        public void onStop() {
            Arm.getInstance().turretHoldLock(false, 0);
        }
    
    }  

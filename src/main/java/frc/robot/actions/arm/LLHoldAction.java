package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.lib.util.TimerBoolean;

public class LLHoldAction extends Action{
        boolean search;
        boolean kill;
        boolean fire;
        TimerBoolean finished = new TimerBoolean(.25);
        double[] vals;

        public LLHoldAction(boolean search) {
            vals = Arm.getInstance().getLLVals();
            this.search = search;
            fire = false;
            kill = false;
        }

        public LLHoldAction(boolean search, boolean kill) {
            vals = Arm.getInstance().getLLVals();
            this.search = search;
            this.kill = kill;
            fire = false;
        }

        public LLHoldAction(boolean search, boolean kill, boolean fire) {
            vals = Arm.getInstance().getLLVals();
            this.search = search;
            this.kill = kill;
            this.fire = fire;
        }

        @Override
        public void onStart() {
            Arm.getInstance().turretHoldLock(true, Arm.getInstance().getTurretEncoder() -  (vals[0] * Constants.TURRET_TPD));
            
        }
    
        @Override
        public void onLoop() {
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
        }
    
    }  

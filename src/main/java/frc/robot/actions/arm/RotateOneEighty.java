package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class RotateOneEighty extends Action{
        double goal;
        double delta;
        boolean unlock;
        public RotateOneEighty(boolean unlock) {
            this.unlock = unlock;
        }

        @Override
        public void onStart() {
            goal = Math.signum(Arm.getInstance().getTurretEncoder()) * 180 * Constants.TURRET_TPD;
            Arm.getInstance().turretHoldLock(true, goal);
            delta = goal - Arm.getInstance().getTurretEncoder();
            Arm.getInstance().clearRamp();
        }
    
        @Override
        public void onLoop() { 
            if((Arm.getInstance().getTurretEncoder() > (goal - delta / 2) && delta > 0)  
            || (Arm.getInstance().getTurretEncoder() < (goal - delta / 2) && delta < 0)) {
                Arm.getInstance().incrRamp(-.037);
            } else {
                Arm.getInstance().incrRamp(.012);
            }
        }
    
        @Override
        public boolean isFinished() {
            return !unlock && Math.abs(Arm.getInstance().getTurretEncoder() - goal) < 500;
        }
    
        @Override
        public void onStop() {
            if(unlock)    
                Arm.getInstance().turretHoldLock(false, 0);
        }
    
    } 

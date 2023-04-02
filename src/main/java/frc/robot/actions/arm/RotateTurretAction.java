package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class RotateTurretAction extends Action{
        double goal;
        double delta;
        boolean unlock;
        public RotateTurretAction(double angle, boolean unlock) {
            this.goal = angle;
            this.unlock = unlock;
        }

        @Override
        public void onStart() {
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
                Arm.getInstance().incrRamp(.018);
            }
        }
    
        @Override
        public boolean isFinished() {
            return !unlock && Math.abs(Arm.getInstance().getTurretEncoder() - goal) < Constants.TURRET_TPD * 2;
        }
    
        @Override
        public void onStop() {
            Arm.getInstance().clearRamp();;
            if(unlock)    
                Arm.getInstance().turretHoldLock(false, 0);
        }
    
    } 

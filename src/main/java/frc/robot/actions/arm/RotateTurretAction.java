package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class RotateTurretAction extends Action{
        double goal;
        double delta;
        int dir;
        public RotateTurretAction(double angle) {
            this.goal = angle;
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
                Arm.getInstance().incrRamp(.01);
            }
        }
    
        @Override
        public boolean isFinished() {
            return Math.abs(Arm.getInstance().getTurretEncoder() - goal) < 500;
        }
    
        @Override
        public void onStop() {
            Arm.getInstance().clearRamp();
        }
    
    } 

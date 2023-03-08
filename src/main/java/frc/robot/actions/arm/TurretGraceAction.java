package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class TurretGraceAction extends Action {
        double angle;
        double delta;
        public TurretGraceAction(double angle) {
            this.angle = angle;
        }

        @Override
        public void onStart() {
            Arm.getInstance().turretHoldLock(true, angle);
        }
    
        @Override
        public void onLoop() { 
        }
    
        @Override
        public boolean isFinished() {
            return false;
        }
    
        @Override
        public void onStop() {
            Arm.getInstance().turretHoldLock(false, 0);
            Arm.getInstance().setTVComp(11);
        }
    
    }  

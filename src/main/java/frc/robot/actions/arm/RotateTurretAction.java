package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class RotateTurretAction extends Action{
        double angle;
        public RotateTurretAction(double angle) {
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
            return true;
        }
    
        @Override
        public void onStop() {
            //Arm.getInstance().turretHoldLock(false, 0);
        }
    
    } 

package frc.robot.actions.arm;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class TurretHoldAction extends Action{
    
        static double max_vel = 40960;
        static double max_accel = 40960;
        double angle;
        double vel;
        double last_time;
        boolean decel;
        public TurretHoldAction(double angle) {
            this.angle = angle;
            last_time = Timer.getFPGATimestamp();
            decel = false;
        }

        @Override
        public void onStart() {
        }
    
        @Override
        public void onLoop() { 
            double cur_time = Timer.getFPGATimestamp();
            vel += (decel ? -1 : 1) * max_accel * (cur_time - last_time);
            vel = Math.min(vel, max_vel);
            Arm.getInstance().turretHoldLock(true, vel);
            decel = Math.abs(Arm.getInstance().getTurretEncoder() - angle) < (max_vel * max_vel / max_accel / 2);
        }
    
        @Override
        public boolean isFinished() {
            return Math.abs(Arm.getInstance().getTurretEncoder() - angle) < 200;
        }
    
        @Override
        public void onStop() {
            Arm.getInstance().turretHoldLock(false, 0);
        }
    
    }  

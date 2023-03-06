package frc.robot.actions.arm;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
public class LLHoldAction extends Action{
        private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-worbots");
        private static NetworkTableEntry tx = table.getEntry("tx");
        private static NetworkTableEntry tv = table.getEntry("tv");

        @Override
        public void onStart() {
            Arm.getInstance().turretHoldLock(true, Arm.getInstance().getTurretEncoder() -  (tx.getDouble(0.0) * Constants.TURRET_TPD));
            System.out.println(tx.getDouble(1000));
            System.out.println(tv.getDouble(1000));
            System.out.println(table.getPath());
        }
    
        @Override
        public void onLoop() {
            Arm.getInstance().turretHoldLock(true, Arm.getInstance().getTurretEncoder() -  (tx.getDouble(0.0) * Constants.TURRET_TPD)); 
        }
    
        @Override
        public boolean isFinished() {
            return tv.getDouble(0.0) < 1;
        }
    
        @Override
        public void onStop() {
            Arm.getInstance().turretHoldLock(false, 0);
        }
    
    }  

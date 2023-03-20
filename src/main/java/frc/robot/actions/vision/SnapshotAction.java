package frc.robot.actions.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class SnapshotAction extends Action {

    @Override
    public void onStart() {
        Arm.snapshots.setNumber(1);
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
        Arm.snapshots.setNumber(0);
        
    }
    
}

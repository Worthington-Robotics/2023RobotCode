package frc.robot.actions.arm;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.VisionLink;
import frc.robot.subsystems.VisionLink.LimelightPipeline;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class LLHoldPipelineAction extends Action{
        private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-worbots");
        private static NetworkTableEntry tx = table.getEntry("tx");
        private static NetworkTableEntry tv = table.getEntry("tv");
        private LimelightPipeline pipeline;

        public LLHoldPipelineAction(LimelightPipeline pipeline) {
            this.pipeline = pipeline;
        }

        @Override
        public void onStart() {
            VisionLink.getInstance().setPipeline(pipeline);
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

package frc.robot.actions.arm;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.control.ErrorChecker;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.VisionLink;
import frc.robot.subsystems.VisionLink.LimelightPipeline;



public class LimelightTurretCorrectionPipelineAction extends Action {
    ErrorChecker checker = new ErrorChecker(Constants.LIMELIGHT_ANGLE_ACCEPTANCE, Constants.LIMELIGHT_ANGLE_PID_MINIMUM_TIME);
	NetworkTableInstance inst = NetworkTableInstance.getDefault();
	NetworkTable limelight = inst.getTable("limelight-worbots");
    LimelightPipeline pipeline;

    public LimelightTurretCorrectionPipelineAction(LimelightPipeline pipe) {
        this.pipeline = pipe;
    }

    @Override
    public void onStart() {
			DoubleTopic txNet = limelight.getDoubleTopic("tx");
			//double heading = Arm.getInstance().getTurretAngle();
			DoubleSubscriber tx = txNet.subscribe(0.0);
            //Arm.getInstance().setDesiredTurret(heading - tx.get());
            VisionLink.getInstance().setPipeline(pipeline);
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
			return checker.check(Arm.getInstance().getTurretError() / Constants.TURRET_ENCODER_PER_DEGREE);
    }

    @Override
    public void onStop() {}
}


package frc.robot.actions.arm;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.statemachine.Action;
import frc.lib.util.TimerBoolean;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Arm.ArmPose;

public class AprilTagCorrectionAction extends Action{

    double referenceDistance = 0;
    double extensionMultiplier = 0;
    
    @Override
    public void onStart() {
        Arm.pipeline.setDefaultDouble(2);
        double[] pose = NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NETWORK_ID).getEntry("targetpose_cameraspace").getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        double aprilTagDist = pose[2]; //in meters
        double deltaDist = referenceDistance - aprilTagDist;
        double extensionChange = deltaDist * extensionMultiplier;

        Arm.getInstance().correctExtension(extensionChange);

    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Arm.getInstance().getExtendEncoderError()) < Constants.EXTENSION_ENCODER_ERROR_ACCEPTANCE;
    }

    @Override
    public void onStop() {
    }
    
}

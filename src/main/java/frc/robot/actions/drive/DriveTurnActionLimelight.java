package frc.robot.actions.drive;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.control.ErrorChecker;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveTurnActionLimelight extends Action {
    ErrorChecker checker = new ErrorChecker(Constants.ANGLE_ACCEPTANCE, Constants.ANGLE_PID_MINIMUM_TIME);
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable lime = inst.getTable(Constants.LIMELIGHT_NETWORK_ID);

    public DriveTurnActionLimelight() {}

    @Override
    public void onStart() {
			DoubleTopic txNet = lime.getDoubleTopic("tx");
			double heading = DriveTrain.getInstance().getHeadingDegrees();
			DoubleSubscriber tx = txNet.subscribe(0.0);
			DriveTrain.getInstance().setTurning(heading - tx.get());
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
			return checker.check(DriveTrain.getInstance().getHeadingError());
    }

    @Override
    public void onStop() {
			DriveTrain.getInstance().resetEncoders();
			DriveTrain.getInstance().setOpenLoop();
    }
}

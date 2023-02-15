package frc.robot.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Dummy;
import frc.robot.Constants;

public class DummyPrint extends Action {
	@Override
	public void onStart() {
		SmartDashboard.putBoolean("Dummy/pressed", true);
	}

	@Override
	public void onLoop() {
		System.out.println("Running");
		System.out.println(Dummy.getInstance().getCount());
	}

	@Override
	public void onStop() {
		System.out.println("Done");
		SmartDashboard.putBoolean("Dummy/pressed", false);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}

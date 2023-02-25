package frc.robot.actions.arm;
	@Override
	public void onLoop() {}

	@Override
	public void onStop() {
		Arm.getInstance().setRawPivotPower(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
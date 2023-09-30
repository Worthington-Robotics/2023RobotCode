package frc.robot.actions.drive;

import java.util.Collections;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveDrive;

public class UpdatePoseEstimator extends CommandBase {
    public void initialize() {
        SwerveDrive.getInstance().setVisionUpdates(true);
    }

    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }

    public void execute() {}

    public boolean isFinished() {
        return true;
    }

    public void end(boolean end) {
        SwerveDrive.getInstance().setVisionUpdates(false);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

};
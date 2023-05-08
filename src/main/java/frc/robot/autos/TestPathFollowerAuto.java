package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.actions.PPSwerveControllerAction;

import edu.wpi.first.math.controller.PIDController;
import frc.lib.pathplanner.PPStateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.drive.AutoPathPlannerClearAction;
import frc.robot.actions.drive.AutoPathPlannerSetAction;
import frc.robot.subsystems.DriveTrain;

public class TestPathFollowerAuto extends PPStateMachineDescriptor {
    PathPlannerTrajectory traj = PathPlanner.loadPath(Constants.DRIVE_TRAJECTORY, Constants.PATH_CONSTRAINTS);

    public TestPathFollowerAuto() {
        setPath(traj);
        addSequential("", new AutoPathPlannerSetAction());
        addSequential("", new PPSwerveControllerAction(traj, DriveTrain.getInstance().getPose(), new PIDController(2.2, 0, 0), new PIDController(2.2, 0, 0), new PIDController(6, 0, 0), DriveTrain.getInstance().setModuleStates(), false));
        addSequential("", new AutoPathPlannerClearAction());
    }
}

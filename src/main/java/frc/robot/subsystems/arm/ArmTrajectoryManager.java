package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.arm.ArmPose.Preset;
import frc.robot.subsystems.arm.ArmTrajectory.Parameters;

public class ArmTrajectoryManager {
    private List<ArmTrajectory> trajectories = new ArrayList<>();
    private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(3, 2).setStartVelocity(0).setEndVelocity(0);

    public ArmTrajectoryManager() {
        System.out.println("Generating " + ArmPose.Preset.values().length * (ArmPose.Preset.values().length-1) + " splines.");

        for(ArmPose.Preset pose : ArmPose.Preset.values()) {
            List<Preset> leftovPresets = new ArrayList<>(); //Get all other poses that arent this one
            for (Preset preset : ArmPose.Preset.values()) {
                if (pose != preset) {
                    leftovPresets.add(preset);
                }
            }

            for(Preset preset : leftovPresets) {
                List<Translation2d> waypoints = new ArrayList<>();
                Trajectory traj = TrajectoryGenerator.generateTrajectory(pose.getPose2d(), waypoints, preset.getPose2d(), trajectoryConfig);
                trajectories.add(trajToArmTraj(pose.getPose2d(), preset.getPose2d(), traj));
            }
        }

        System.out.println("Done generating splines. Total: " + trajectories.size());
    }

    public ArmTrajectory splineToTraj(Pose2d initialPose, Pose2d finalPose, double time, CubicHermiteSpline spline) {
        ArmTrajectory trajectory = new ArmTrajectory(new Parameters(ArmKinematics.inverseSafe(initialPose), ArmKinematics.inverseSafe(finalPose)));

        List<Vector<N3>> points = new ArrayList<>();
        for(double i = 0; i<1.0; i+=0.02) {
            points.add(ArmKinematics.inverseSafe(spline.getPoint(i).poseMeters));
        }
        trajectory.setPoints(time, points);
        return trajectory;
    }

    public ArmTrajectory trajToArmTraj(Pose2d initialPose, Pose2d finalPose, Trajectory traj) {
        ArmTrajectory trajectory = new ArmTrajectory(new Parameters(ArmKinematics.inverseSafe(initialPose), ArmKinematics.inverseSafe(finalPose)));

        List<Vector<N3>> points = new ArrayList<>();
        for(double i = 0; i<traj.getTotalTimeSeconds(); i+=0.02) {
            points.add(ArmKinematics.inverseSafe(traj.sample(i).poseMeters));
        }
        trajectory.setPoints(traj.getTotalTimeSeconds(), points);
        return trajectory;
    }

    public ArmTrajectory getTrajectory(int index) {
        return trajectories.get(index);
    }

    public ArmTrajectory getTrajectory(Pose2d initialPose, Pose2d finalPose) {
        return trajectories.get(0);
    }
}

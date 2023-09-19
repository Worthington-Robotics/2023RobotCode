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
import frc.robot.subsystems.arm.ArmPose.Preset;
import frc.robot.subsystems.arm.ArmTrajectory.Parameters;

public class ArmTrajectoryManager {
    private List<ArmTrajectory> trajectories = new ArrayList<>();

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
                ControlVector[] vecs = SplineHelper.getCubicControlVectorsFromWaypoints(pose.getPose2d(), new Translation2d[] {}, preset.getPose2d());
                CubicHermiteSpline[] splines = SplineHelper.getCubicSplinesFromControlVectors(vecs[0], new Translation2d[] {}, vecs[1]);
                trajectories.add(splineToTraj(pose.getPose2d(), preset.getPose2d(), 1.0, splines[0]));
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

    public ArmTrajectory getTrajectory(int index) {
        return trajectories.get(index);
    }

    public ArmTrajectory getTrajectory(Pose2d initialPose, Pose2d finalPose) {
        return trajectories.get(0);
    }
}

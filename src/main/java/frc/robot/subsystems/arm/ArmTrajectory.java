package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public class ArmTrajectory {
    private double totalTime = 0.0;
    private Parameters parameters;
    private List<Pose2d> points = new ArrayList<>();

    public static class Parameters {
        private final Pose2d initialPose;
        private final Pose2d finalPose;

        public Parameters(Pose2d initialPose, Pose2d finalPose) {
            this.initialPose = initialPose;
            this.finalPose = finalPose;
        }

        public Pose2d initialPose() {
            return initialPose;
        }

        public Pose2d finalPose() {
            return finalPose;
        }
    }

    public ArmTrajectory(Parameters parameters) {
        this.parameters = parameters;
        points.add(parameters.finalPose());
        points.add(parameters.initialPose());
    }

    public Parameters getParameters() {
        return parameters;
    }

    public boolean isGenerated() {
        return totalTime > 0.0 && points.size() > 2;
    }

    public void setPoints(double totalTime, List<Pose2d> points) {
        this.totalTime = totalTime;
        this.points = points;
    }

    public double getTotalTime() {
        return totalTime;
    }

    public List<Pose2d> getPoints() {
        return this.points;
    }

    public Pose2d sample(double timestamp) {
        if (timestamp >= totalTime) {
            return parameters.finalPose();
        } else if (timestamp <= 0.0) {
            return parameters.initialPose();
        } else {
            double percentage = timestamp/totalTime;
            int closestValue = (int)(points.size() * percentage);
            return points.get(closestValue);
        }
    }
}

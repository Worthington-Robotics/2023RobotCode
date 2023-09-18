package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class ArmTrajectory {
    private double totalTime = 0.0;
    private Parameters parameters;
    private List<Vector<N3>> points = new ArrayList<>();

    public static class Parameters {
        private final Vector<N3> initialPose;
        private final Vector<N3> finalPose;

        public Parameters(Vector<N3> initialPose, Vector<N3> finalPose) {
            this.initialPose = initialPose;
            this.finalPose = finalPose;
        }

        public Vector<N3> initialPose() {
            return initialPose;
        }

        public Vector<N3> finalPose() {
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

    public void setPoints(double totalTime, List<Vector<N3>> points) {
        this.totalTime = totalTime;
        this.points = points;
    }

    public double getTotalTime() {
        return totalTime;
    }

    public List<Vector<N3>> getPoints() {
        return this.points;
    }

    public Vector<N3> sample(double timestamp) {
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

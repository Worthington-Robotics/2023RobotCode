package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.VecBuilder;
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

    public void setTotalTime(double totalTime) {
        this.totalTime = totalTime;
    }

    public double getTotalTime() {
        return totalTime;
    }

    public List<Vector<N3>> getPoints() {
        return this.points;
    }

    public Vector<N3> sample(double timestamp) {
        // if (timestamp >= totalTime) {
        //     return parameters.finalPose();
        // } else if (timestamp <= 0.0) {
        //     return parameters.initialPose();
        // } else {
        //     double percentage = timestamp/totalTime;
        //     int closestValue = (int)(points.size() * percentage);
        //     return points.get(closestValue);
        // }
        if (timestamp >= totalTime) {
            return parameters.finalPose();
        } else if (timestamp <= 0.0) {
            return parameters.initialPose();
        } else {
            double percentage = timestamp/totalTime;
            double wristDelta = parameters.finalPose().get(2, 0) - parameters.initialPose().get(2, 0);
            int closestValue = (int)(points.size() * percentage);
            Vector<N3> pose = VecBuilder.fill(points.get(closestValue).get(0, 0), points.get(closestValue).get(1, 0), (wristDelta*percentage)+parameters.initialPose().get(2, 0));
            return pose;
        }
    }

    // public Vector<N3> sampleInterpolateWrist(double timestamp) {
    //     if (timestamp >= totalTime) {
    //         return parameters.finalPose();
    //     } else if (timestamp <= 0.0) {
    //         return parameters.initialPose();
    //     } else {
    //         double percentage = timestamp/totalTime;
    //         double wristDelta = parameters.finalPose().get(2, 0) - parameters.initialPose().get(2, 0);
    //         int closestValue = (int)(points.size() * percentage);
    //         Vector<N3> pose = VecBuilder.fill(points.get(closestValue).get(0, 0), points.get(closestValue).get(1, 0), (wristDelta*percentage)+parameters.initialPose().get(2, 0));
    //         return pose;
    //     }
    // }
}

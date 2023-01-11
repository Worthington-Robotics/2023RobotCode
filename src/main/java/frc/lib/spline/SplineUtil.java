package frc.lib.spline;

import java.util.ArrayList;
import java.util.List;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;

public class SplineUtil {
    /**
     * creates an optimized spilne form a list of poses
     * @param waypoints -- list of pose2D
     * @param maxDx -- maximum change in X
     * @param maxDy -- maximum change in Y
     * @param maxDTheta -- maximum change in angle
     * @return a trajectory generated against a set of splines
     */
    public static List<Pose2dWithCurvature> pathFromSplineWaypoints(final List<Pose2d> waypoints,
                                                                    double maxDx, double maxDy, double maxDTheta) {
        List<QuinticHermiteSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new QuinticHermiteSpline(waypoints.get(i - 1), waypoints.get(i)));
        }
        QuinticHermiteSpline.optimizeSpline(splines);
        return trajectoryFromSplines(splines, maxDx, maxDy, maxDTheta);
    }

    public static List<Pose2dWithCurvature> trajectoryFromSplines(final List<? extends Spline> splines,
                                                                        double maxDx, double maxDy, double maxDTheta) {
        return SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta);
    }
}
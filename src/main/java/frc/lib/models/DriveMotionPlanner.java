package frc.lib.models;

import frc.lib.geometry.ITranslation2d;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.physics.DCMotorTransmission;
import frc.lib.physics.DifferentialDrive;
import frc.lib.spline.SplineUtil;
import frc.lib.trajectory.*;
import frc.lib.trajectory.constraint.*;
import frc.lib.util.CSVWritable;
import frc.lib.util.Util;
import frc.robot.Constants;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class DriveMotionPlanner implements CSVWritable {
    private static final double kMaxDx = 0.02; // meters
    private static final double kMaxDy = 0.02; // meters
    private static final double kMaxDTheta = Math.toRadians(2.0);

    public enum FollowerType {
        FEEDFORWARD_ONLY, 
        PURE_PURSUIT, 
        PID, 
        NONLINEAR_FEEDBACK
    }

    FollowerType mFollowerType = FollowerType.NONLINEAR_FEEDBACK;

    public void setFollowerType(FollowerType type) {
        mFollowerType = type;
    }

    final DifferentialDrive mModel;

    Trajectory mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY, mStartTime = Double.POSITIVE_INFINITY;
    TimedState mSetpoint = new TimedState();
    Pose2d mError = Pose2d.identity();
    Output mOutput = new Output();

    DifferentialDrive.ChassisState prev_velocity_ = new DifferentialDrive.ChassisState();
    double mDt = 0.0;

    public DriveMotionPlanner() {
        //speed per volt: rad/s per volt 
        //torque per volt: m*m*kg*rad/s^2 /2 V
        //friction voltage: V
        final DCMotorTransmission transmission = new DCMotorTransmission(Constants.DRIVE_Kv,
         Constants.DRIVE_WHEEL_RADIUS * Constants.DRIVE_WHEEL_RADIUS * Constants.ROBOT_LINEAR_INERTIA * Constants.DRIVE_Ka/ 
         2.0, Constants.DRIVE_V_INTERCEPT);
        mModel = new DifferentialDrive(Constants.ROBOT_LINEAR_INERTIA, Constants.ROBOT_ANGULAR_INERTIA,
                Constants.ROBOT_ANGULAR_DRAG, Constants.DRIVE_WHEEL_RADIUS, Constants.DRIVE_WHEEL_TRACK_WIDTH / 2.0 * 
                Constants.TRACK_SCRUB_FACTOR, transmission, transmission);
    }

    public void setTrajectory(final Trajectory trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.sample(0);
        for (TimedState state : mCurrentTrajectory.getStates()) {
            if (state.velocity > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (state.velocity < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
    }

    public void reset() {
        mError = Pose2d.identity();
        mOutput = new Output();
        mLastTime = mStartTime = Double.POSITIVE_INFINITY;
    }

    /**
     * hooks the trajectory generator to create a trajectory that starts and ends with zero velocity
     * @param reversed whether the robot should run this trajectory backwards
     * @param waypoints the list of pose2ds the robot should path through
     * @param constraints any addtional contraints the trajectory generator should follow
     * @param max_vel the maximum allowable velocity the robot can travel (m / s)
     * @param max_accel the maximum allowable acceleration the robot can experince (m / s^2)
     * @param max_voltage the maximum allowable voltage the robot can use
     * @return a complete trajectory, ready for execution
     */
    public Trajectory generateTrajectory(boolean reversed, final List<Pose2d> waypoints,
            final List<TrajectoryConstraint> constraints, double max_vel, 
            double max_accel, 
            double max_voltage) {
                Trajectory tra;
                if(reversed)
                {
                    tra = generateRevTrajectory(waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
                }
                else
                {
                    tra = generateTrajectory(waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
                }
        return tra;
    }

    /**
     * hooks the trajectory generator to create a trajectory with pre-imposed limits
     * @param reversed whether the robot should run this trajectory backwards
     * @param waypoints the list of pose2ds the robot should path through
     * @param constraints any addtional contraints the trajectory generator should follow
     * @param start_vel the starting velocity of the robot (m / s)
     * @param end_vel the ending velocity of the robot (m / s)
     * @param max_vel the maximum allowable velocity the robot can travel (m / s)
     * @param max_accel the maximum allowable acceleration the robot can experince (m / s^2)
     * @param max_voltage the maximum allowable voltage the robot can use
     * @return a complete trajectory, ready for execution
     */
    public Trajectory generateTrajectory(final List<Pose2d> waypoints,
            final List<TrajectoryConstraint> constraints, double start_vel, double end_vel, double max_vel, // inches/s
            double max_accel, 
            double max_voltage) {
        // Create a trajectory from splines.
        List<Pose2dWithCurvature> path = SplineUtil.pathFromSplineWaypoints(waypoints, kMaxDx, kMaxDy, kMaxDTheta);
        // Create the constraint that the robot must be able to traverse the trajectory
        // without ever applying more
        // than the specified voltage.
        final DifferentialDriveDynamicsConstraint drive_constraints = new DifferentialDriveDynamicsConstraint(mModel,
                max_voltage);
        List<TrajectoryConstraint> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }
        // Generate the timed trajectory.
        Trajectory timed_trajectory = TrajectoryParameterizer.timeParameterizeTrajectory(path, all_constraints, start_vel,
                end_vel, max_vel, max_accel, false);
        return timed_trajectory;
    }

     /**
     * hooks the trajectory generator to create a trajectory with pre-imposed limits
     * @param reversed whether the robot should run this trajectory backwards
     * @param waypoints the list of pose2ds the robot should path through
     * @param constraints any addtional contraints the trajectory generator should follow
     * @param start_vel the starting velocity of the robot (m / s)
     * @param end_vel the ending velocity of the robot (m / s)
     * @param max_vel the maximum allowable velocity the robot can travel (m / s)
     * @param max_accel the maximum allowable acceleration the robot can experince (m / s^2)
     * @param max_voltage the maximum allowable voltage the robot can use
     * @return a complete trajectory, ready for execution
     */
    public Trajectory generateRevTrajectory(final List<Pose2d> waypoints,
            final List<TrajectoryConstraint> constraints, double start_vel, double end_vel, double max_vel, // inches/s
            double max_accel, 
            double max_voltage) {
        
        final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
            ArrayList<Pose2d> waypointsFlipped = new ArrayList<>(waypoints.size());
            for (int i = 0; i < waypoints.size(); ++i) {
                waypointsFlipped.add(waypoints.get(i).transformBy(flip));
            }
        // Create a trajectory from splines.
            List<Pose2dWithCurvature> path = SplineUtil.pathFromSplineWaypoints(waypointsFlipped, kMaxDx, kMaxDy, kMaxDTheta);
            List<Pose2dWithCurvature> flipped = new ArrayList<>(path.size());
            for (int i = 0; i < path.size(); ++i) {
                flipped.add(new Pose2dWithCurvature(path.get(i).getPose().transformBy(flip),
                        path.get(i).getCurvature(), path.get(i).getDCurvatureDs()));
            }
            path = new ArrayList<>(flipped);
        // Create the constraint that the robot must be able to traverse the trajectory
        // without ever applying more
        // than the specified voltage.
        final DifferentialDriveDynamicsConstraint drive_constraints = new DifferentialDriveDynamicsConstraint(mModel,
                max_voltage);
        List<TrajectoryConstraint> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }
        // Generate the timed trajectory.
        Trajectory timed_trajectory = TrajectoryParameterizer.timeParameterizeTrajectory(path, all_constraints, start_vel,
                end_vel, max_vel, max_accel, true);
        return timed_trajectory;
    }

    public static class Output {
        public Output() {
        }

        public Output(double left_velocity, double right_velocity, double left_accel, double right_accel,
                double left_feedforward_voltage, double right_feedforward_voltage) {
            this.left_velocity = left_velocity;
            this.right_velocity = right_velocity;
            this.left_accel = left_accel;
            this.right_accel = right_accel;
            this.left_feedforward_voltage = left_feedforward_voltage;
            this.right_feedforward_voltage = right_feedforward_voltage;
        }

        public double left_velocity; // rad/s
        public double right_velocity; // rad/s

        public double left_accel; // rad/s^2
        public double right_accel; // rad/s^2

        public double left_feedforward_voltage;
        public double right_feedforward_voltage;

        public void flip() {
            double tmp_left_velocity = left_velocity;
            left_velocity = -right_velocity;
            right_velocity = -tmp_left_velocity;

            double tmp_left_accel = left_accel;
            left_accel = -right_accel;
            right_accel = -tmp_left_accel;

            double tmp_left_feedforward = left_feedforward_voltage;
            left_feedforward_voltage = -right_feedforward_voltage;
            right_feedforward_voltage = -tmp_left_feedforward;
        }
    }

    protected Output updatePID(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
        DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState();
        // Feedback on longitudinal error (distance).
        final double kPathKX = 5.0;
        final double kPathKY = 1.0;
        final double kPathKTheta = 5.0;
        adjusted_velocity.linear = dynamics.chassis_velocity.linear
                + kPathKX * mError.getTranslation().x();
        adjusted_velocity.angular = dynamics.chassis_velocity.angular
                + dynamics.chassis_velocity.linear * kPathKY * mError.getTranslation().y()
                + kPathKTheta * mError.getRotation().getRadians();

        double curvature = adjusted_velocity.angular / adjusted_velocity.linear;
        if (Double.isInfinite(curvature)) {
            adjusted_velocity.linear = 0.0;
            adjusted_velocity.angular = dynamics.chassis_velocity.angular;
        }

        // Compute adjusted left and right wheel velocities.
        final DifferentialDrive.WheelState wheel_velocities = mModel.solveInverseKinematics(adjusted_velocity);
        final double left_voltage = dynamics.voltage.left
                + (wheel_velocities.left - dynamics.wheel_velocity.left) / mModel.left_transmission().speed_per_volt();
        final double right_voltage = dynamics.voltage.right + (wheel_velocities.right - dynamics.wheel_velocity.right)
                / mModel.right_transmission().speed_per_volt();

        return new Output(wheel_velocities.left, wheel_velocities.right, dynamics.wheel_acceleration.left,
                dynamics.wheel_acceleration.right, left_voltage, right_voltage);
    }

    // TODO fix this mess
    protected Output updatePurePursuit(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
        double lookahead_time = Constants.PATH_LOOK_AHEAD_TIME;
        final double kLookaheadSearchDt = 0.01;
        TimedState lookahead_state = mCurrentTrajectory.sample(lookahead_time);
        double actual_lookahead_distance = mSetpoint.pose.distance(lookahead_state.pose);
        while (actual_lookahead_distance < Constants.PATH_MIN_LOOK_AHEAD_DISTANCE
                && mLastTime - mStartTime > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.sample(mLastTime + lookahead_time);
            actual_lookahead_distance = mSetpoint.pose.distance(lookahead_state.pose);
        }
        if (actual_lookahead_distance < Constants.PATH_MIN_LOOK_AHEAD_DISTANCE) {
            lookahead_state = new TimedState(lookahead_state.time, lookahead_state.velocity,
                    lookahead_state.acceleration,
                    new Pose2d(lookahead_state.pose.transformBy(Pose2d.fromTranslation(new Translation2d(
                    (mIsReversed ? -1.0 : 1.0) * (Constants.PATH_MIN_LOOK_AHEAD_DISTANCE - actual_lookahead_distance),
                    0.0)))), 0.0);
        }

        DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState();
        // Feedback on longitudinal error (distance).
        adjusted_velocity.linear = dynamics.chassis_velocity.linear
                + Constants.Path_Kx * mError.getTranslation().x();

        // Use pure pursuit to peek ahead along the trajectory and generate a new
        // curvature.
        final Arc<TimedState> arc = new Arc<>(current_state, lookahead_state);

        double curvature = 1.0 / arc.radius;
        if (Double.isInfinite(curvature)) {
            adjusted_velocity.linear = 0.0;
            adjusted_velocity.angular = dynamics.chassis_velocity.angular;
        } else {
            adjusted_velocity.angular = curvature * dynamics.chassis_velocity.linear;
        }

        dynamics.chassis_velocity = adjusted_velocity;
        dynamics.wheel_velocity = mModel.solveInverseKinematics(adjusted_velocity);
        return new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration.left,
                dynamics.wheel_acceleration.right, dynamics.voltage.left, dynamics.voltage.right);
    }

    protected Output updateNonlinearFeedback(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
        // Implements eqn. 5.12 from
        // https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
        final double kBeta = 4; // >0.
        final double kZeta = .75; // Damping coefficient, [0, 1].

        // Compute gain parameter.
        final double k = 2.0 * kZeta
                * Math.sqrt(kBeta * dynamics.chassis_velocity.linear * dynamics.chassis_velocity.linear
                        + dynamics.chassis_velocity.angular * dynamics.chassis_velocity.angular);

        // Compute error components.
        final double angle_error_rads = mError.getRotation().getRadians();
        final double sin_x_over_x = Util.epsilonEquals(angle_error_rads, 0.0, 1E-2) ? 1.0
                : mError.getRotation().sin() / angle_error_rads;
        final DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState(
                dynamics.chassis_velocity.linear * mError.getRotation().cos()
                        + k * mError.getTranslation().x(),
                dynamics.chassis_velocity.angular + k * angle_error_rads + dynamics.chassis_velocity.linear * kBeta
                        * sin_x_over_x * mError.getTranslation().y());

        // Compute adjusted left and right wheel velocities.
        dynamics.chassis_velocity = adjusted_velocity;
        dynamics.wheel_velocity = mModel.solveInverseKinematics(adjusted_velocity);

        dynamics.chassis_acceleration.linear = mDt == 0 ? 0.0
                : (dynamics.chassis_velocity.linear - prev_velocity_.linear) / mDt;
        dynamics.chassis_acceleration.angular = mDt == 0 ? 0.0
                : (dynamics.chassis_velocity.angular - prev_velocity_.angular) / mDt;

        prev_velocity_ = dynamics.chassis_velocity;

        DifferentialDrive.WheelState feedforward_voltages = mModel.solveInverseDynamics(dynamics.chassis_velocity,
                dynamics.chassis_acceleration).voltage;

        return new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration.left,
                dynamics.wheel_acceleration.right, feedforward_voltages.left, feedforward_voltages.right);
    }

    public Output update(double timestamp, Pose2d current_state) {
        if (mCurrentTrajectory == null)
            return new Output();

        if (!Double.isFinite(mStartTime)) {
            mStartTime = timestamp;
        }

        if (mCurrentTrajectory.getTotalTimeSeconds() == (timestamp - mStartTime) && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        mSetpoint = mCurrentTrajectory.sample(timestamp - mStartTime);

        if (mCurrentTrajectory.getTotalTimeSeconds() >= (timestamp - mStartTime)) {
            // Generate feedforward voltages.
            final double velocity_m = mSetpoint.velocity;
            final double curvature_m = mSetpoint.curvature;
            final double dcurvature_ds_m = mSetpoint.dCurvatureDs;
            final double acceleration_m = mSetpoint.acceleration;
            final DifferentialDrive.DriveDynamics dynamics = mModel.solveInverseDynamics(
                    new DifferentialDrive.ChassisState(velocity_m, velocity_m * curvature_m),
                    new DifferentialDrive.ChassisState(acceleration_m, acceleration_m * curvature_m + 
                        velocity_m * velocity_m * dcurvature_ds_m));
            mError = current_state.inverse().transformBy(mSetpoint.pose);

            if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
                mOutput = new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right,
                        dynamics.wheel_acceleration.left, dynamics.wheel_acceleration.right, dynamics.voltage.left,
                        dynamics.voltage.right);
            } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
                mOutput = updatePurePursuit(dynamics, current_state);
            } else if (mFollowerType == FollowerType.PID) {
                mOutput = updatePID(dynamics, current_state);
            } else if (mFollowerType == FollowerType.NONLINEAR_FEEDBACK) {
                mOutput = updateNonlinearFeedback(dynamics, current_state);
            }
        } else {
            // TODO Possibly switch to a pose stabilizing controller?
            mOutput = new Output();
        }
        return mOutput;
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.getTotalTimeSeconds() <= (mLastTime - mStartTime);
    }

    public Pose2d error() {
        return mError;
    }

    public TimedState setpoint() {
        return mSetpoint;
    }

    @Override
    public String toCSV() {
        DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(mOutput.left_velocity) + "," + fmt.format(mOutput.right_velocity) + ","
                + fmt.format(mOutput.left_feedforward_voltage) + "," + fmt.format(mOutput.right_feedforward_voltage)
                + "," + mSetpoint.toCSV();
    }

    @Override
    public int getNumFields() {
        return 4 + mSetpoint.getNumFields();
    }

    @Override
    public String toString(){
        DecimalFormat fmt = new DecimalFormat("#0.000");
        return "left vel m/s: " + fmt.format(mOutput.left_velocity) + ", right vel m/s" + fmt.format(mOutput.right_velocity)
         + ", left ff V: " + fmt.format(mOutput.left_feedforward_voltage) + ", right ff V: " + fmt.format(mOutput.right_feedforward_voltage)
                + ", Setpoint: " + mSetpoint.toString();
    }

    public static class Arc<S extends ITranslation2d<S>> {
        public Translation2d center;
        public double radius;
        public double length;
        public Arc(final Pose2d pose, final S point) {
            center = findCenter(pose, point);
            radius = new Translation2d(center, point.getTranslation()).norm();
            length = findLength(pose, point, center, radius);
            radius *= getDirection(pose, point);
        }

        protected Translation2d findCenter(Pose2d pose, S point) {
            final Translation2d poseToPointHalfway = pose.getTranslation().interpolate(point.getTranslation(), 0.5);
            final Rotation2d normal = pose.getTranslation().inverse().translateBy(poseToPointHalfway).direction()
                    .normal();
            final Pose2d perpendicularBisector = new Pose2d(poseToPointHalfway, normal);
            final Pose2d normalFromPose = new Pose2d(pose.getTranslation(),
                    pose.getRotation().normal());
            if (normalFromPose.isColinear(perpendicularBisector.normal())) {
                // Special case: center is poseToPointHalfway.
                return poseToPointHalfway;
            }
            return normalFromPose.intersection(perpendicularBisector);
        }

        protected double findLength(Pose2d pose, S point, Translation2d center, double radius) {
            if (radius < Double.MAX_VALUE) {
                final Translation2d centerToPoint = new Translation2d(center, point.getTranslation());
                final Translation2d centerToPose = new Translation2d(center, pose.getTranslation());
                // If the point is behind pose, we want the opposite of this angle. To determine if the point is behind,
                // check the sign of the cross-product between the normal vector and the vector from pose to point.
                final boolean behind = Math.signum(
                        Translation2d.cross(pose.getRotation().normal().toTranslation(),
                                new Translation2d(pose.getTranslation(), point.getTranslation()))) > 0.0;
                final Rotation2d angle = Translation2d.getAngle(centerToPose, centerToPoint);
                return radius * (behind ? 2.0 * Math.PI - Math.abs(angle.getRadians()) : Math.abs(angle.getRadians()));
            } else {
                return new Translation2d(pose.getTranslation(), point.getTranslation()).norm();
            }
        }

        protected static <S extends ITranslation2d<S>> double getDirection(Pose2d pose, S point) {
            Translation2d poseToPoint = new Translation2d(pose.getTranslation(), point.getTranslation());
            Translation2d robot = pose.getRotation().toTranslation();
            double cross = robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x();
            return (cross < 0.) ? -1. : 1.; // if robot < pose turn left
        }
    }
}
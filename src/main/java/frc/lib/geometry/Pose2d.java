package frc.lib.geometry;

import frc.lib.util.Util;

/**
 * Represents a 2d pose (rigid transform) containing translational and
 * rotational elements.
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class Pose2d implements IPose2d<Pose2d> {
    protected static final Pose2d kIdentity = new Pose2d();

    public static final Pose2d identity() {
        return kIdentity;
    }

    private final static double kEps = 1E-9;

    protected final Translation2d translation_;
    protected final Rotation2d rotation_;

    public Pose2d() {
        translation_ = new Translation2d();
        rotation_ = new Rotation2d();
    }

    public Pose2d(double x, double y, final Rotation2d rotation) {
        translation_ = new Translation2d(x, y);
        rotation_ = rotation;
    }

    public Pose2d(final Translation2d translation, final Rotation2d rotation) {
        translation_ = translation;
        rotation_ = rotation;
    }

    public Pose2d(final Pose2d other) {
        translation_ = new Translation2d(other.translation_);
        rotation_ = new Rotation2d(other.rotation_);
    }

    public static Pose2d fromTranslation(final Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    public static Pose2d fromRotation(final Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    /**
     * Obtain a new Pose2d from a (constant curvature) velocity.
     *
     * <p>
     * See <a href="https://file.tavsys.net/control/state-space-guide.pdf"> Controls
     * Engineering in the FIRST Robotics Competition</a> section on nonlinear pose
     * estimation for derivation.
     *
     * <p>
     * The twist is a change in pose in the robot's coordinate frame since the
     * previous pose update. When the user runs exp() on the previous known
     * field-relative pose with the argument being the twist, the user will receive
     * the new field-relative pose.
     *
     * <p>
     * "Exp" represents the pose exponential, which is solving a differential
     * equation moving the pose forward in time.
     *
     * @param twist The change in pose in the robot's coordinate frame since the
     *              previous pose update. For example, if a non-holonomic robot
     *              moves forward 0.01 meters and changes angle by 0.5 degrees since
     *              the previous pose update, the twist would be Twist2d{0.01, 0.0,
     *              toRadians(0.5)}
     * @return The new pose of the robot.
     **/
    public Pose2d exp(Twist2d twist) {
        double dx = twist.dx;
        double dy = twist.dy;
        double dtheta = twist.dtheta;

        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);

        double s;
        double c;
        if (Math.abs(dtheta) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        } else {
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }
        Pose2d transform = new Pose2d(new Translation2d(dx * s - dy * c, dx * c + dy * s),
                new Rotation2d(cosTheta, sinTheta, true));

        return this.plus(transform);
    }

    /**
     * Returns a Twist2d that maps this pose to the end pose. If c is the output of
     * a.Log(b), then a.Exp(c) would yield b.
     *
     * @param end The end pose for the transformation.
     * @return The twist that maps this to end.
     */
    public static Twist2d log(final Pose2d transform) {
        // heading assigned to pose
        final double dtheta = transform.getRotation().getRadians();
        // half of heading
        final double half_dtheta = 0.5 * dtheta;

        final double cos_minus_one = transform.getRotation().cos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().sin()) / cos_minus_one;
        }
        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
        return new Twist2d(translation_part.x(), translation_part.y(), dtheta);
    }

    /**
     * Returns the other pose relative to the current pose.
     *
     * <p>
     * This function can often be used for trajectory tracking or pose stabilization
     * algorithms to get the error between the reference and the current pose.
     *
     * @param other The pose that is the origin of the new coordinate frame that the
     *              current pose will be converted into.
     * @return The current pose relative to the new origin pose.
     */
    public Pose2d relativeTo(Pose2d other) {
        // We are rotating the difference between the translations
        // using a clockwise rotation matrix. This transforms the global
        // delta into a local delta (relative to the initial pose).
        final Translation2d m_translation = getTranslation().minus(other.getTranslation())
                .rotateBy(other.getRotation().unaryMinus());

        final Rotation2d m_rotation = getRotation().minus(other.getRotation());
        return new Pose2d(m_translation, m_rotation);
    }

    @Override
    public Translation2d getTranslation() {
        return translation_;
    }

    @Override
    public Rotation2d getRotation() {
        return rotation_;
    }

    /**
     * Transforming this RigidTransform2d means first translating by
     * other.translation and then rotating by other.rotation
     *
     * @param other The other transform.
     * @return This transform * other
     */
    @Override
    public Pose2d transformBy(final Pose2d other) {
        return new Pose2d(translation_.translateBy(other.translation_.rotateBy(rotation_)),
                rotation_.rotateBy(other.rotation_));
    }

    /**
     * The inverse of this transform "undoes" the effect of translating by this
     * transform.
     *
     * @return The opposite of this transform.
     */
    public Pose2d inverse() {
        Rotation2d rotation_inverted = rotation_.inverse();
        return new Pose2d(translation_.inverse().rotateBy(rotation_inverted), rotation_inverted);
    }

    public Pose2d normal() {
        return new Pose2d(translation_, rotation_.normal());
    }

    /**
     * Finds the point where the heading of this pose intersects the heading of
     * another. Returns (+INF, +INF) if parallel.
     */
    public Translation2d intersection(final Pose2d other) {
        final Rotation2d other_rotation = other.getRotation();
        if (rotation_.isParallel(other_rotation)) {
            // Lines are parallel.
            return new Translation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        if (Math.abs(rotation_.cos()) < Math.abs(other_rotation.cos())) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    /**
     * Return true if this pose is (nearly) colinear with the another.
     */
    public boolean isColinear(final Pose2d other) {
        if (!getRotation().isParallel(other.getRotation()))
            return false;
        final Twist2d twist = log(inverse().transformBy(other));
        return (Util.epsilonEquals(twist.dy, 0.0) && Util.epsilonEquals(twist.dtheta, 0.0));
    }

    /**
     * Transforms the pose by the given transformation and returns the new
     * transformed pose.
     *
     * <p>
     * The matrix multiplication is as follows [x_new] [cos, -sin, 0][transform.x]
     * [y_new] += [sin, cos, 0][transform.y] [t_new] [0, 0, 1][transform.t]
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    public Pose2d plus(Pose2d other) {
        return transformBy(other);
    }

    /**
     * Returns the Transform2d that maps the one pose to another.
     *
     * @param other The initial pose of the transformation.
     * @return The transform that maps the other pose to the current pose.
     */
    public Pose2d minus(Pose2d other) {
        final Pose2d pose = this.relativeTo(other);
        return new Pose2d(pose.getTranslation(), pose.getRotation());
    }

    public boolean epsilonEquals(final Pose2d other, double epsilon) {
        return getTranslation().epsilonEquals(other.getTranslation(), epsilon)
                && getRotation().isParallel(other.getRotation());
    }

    private static Translation2d intersectionInternal(final Pose2d a, final Pose2d b) {
        final Rotation2d a_r = a.getRotation();
        final Rotation2d b_r = b.getRotation();
        final Translation2d a_t = a.getTranslation();
        final Translation2d b_t = b.getTranslation();

        final double tan_b = b_r.tan();
        final double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y()) / (a_r.sin() - a_r.cos() * tan_b);
        if (Double.isNaN(t)) {
            return new Translation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        return a_t.translateBy(a_r.toTranslation().scale(t));
    }

    /**
     * Do twist interpolation of this pose assuming constant curvature.
     */
    @Override
    public Pose2d interpolate(final Pose2d other, double x) {
        if (x <= 0) {
            return new Pose2d(this);
        } else if (x >= 1) {
            return new Pose2d(other);
        }
        final Twist2d twist = Pose2d.log(inverse().transformBy(other));
        return exp(twist.scaled(x));
    }

    @Override
    public String toString() {
        return "T:" + translation_.toString() + ", R:" + rotation_.toString();
    }

    @Override
    public String toCSV() {
        return translation_.toCSV() + "," + rotation_.toCSV();
    }

    @Override
    public int getNumFields() {
        return translation_.getNumFields() + rotation_.getNumFields();
    }

    @Override
    public double distance(final Pose2d other) {
        return Pose2d.log(inverse().transformBy(other)).norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Pose2d))
            return false;
        return epsilonEquals((Pose2d) other, Util.kEpsilon);
    }

    @Override
    public Pose2d getPose() {
        return this;
    }

    @Override
    public Pose2d mirror() {
        return new Pose2d(new Translation2d(getTranslation().x(), -getTranslation().y()), getRotation().inverse());
    }
}

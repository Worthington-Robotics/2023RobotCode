package frc.lib.geometry;

import frc.lib.util.Util;

import java.text.DecimalFormat;

/**
 * A translation in a 2d coordinate frame. Translations are simply shifts in an
 * (x, y) plane.
 */
public class Translation2d implements ITranslation2d<Translation2d> {
    protected static final Translation2d kIdentity = new Translation2d();

    public static final Translation2d identity() {
        return kIdentity;
    }

    protected final double x_;
    protected final double y_;

    public Translation2d() {
        x_ = 0;
        y_ = 0;
    }

    public Translation2d(double x, double y) {
        x_ = x;
        y_ = y;
    }

    public Translation2d(final Translation2d other) {
        x_ = other.x_;
        y_ = other.y_;
    }

    public Translation2d(final Translation2d start, final Translation2d end) {
        x_ = end.x_ - start.x_;
        y_ = end.y_ - start.y_;
    }

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     *
     * @return sqrt(x ^ 2 + y ^ 2)
     */
    public double norm() {
        return Math.hypot(x_, y_);
    }

    public double norm2() {
        return x_ * x_ + y_ * y_;
    }

    public double x() {
        return x_;
    }

    public double y() {
        return y_;
    }

    /**
     * Adds two translations in 2d space and returns the sum. This is similar to
     * vector addition.
     *
     * <p>
     * For example, Translation2d{1.0, 2.5} + Translation2d{2.0, 5.5} =
     * Translation2d{3.0, 8.0}
     *
     * @param other The translation to add.
     * @return The sum of the translations.
     */
    public Translation2d translateBy(final Translation2d other) {
        return new Translation2d(x_ + other.x_, y_ + other.y_);
    }

    /**
     * Applies a rotation to the translation in 2d space.
     *
     * <p>
     * This multiplies the translation vector by a counterclockwise rotation matrix
     * of the given angle. [x_new] [other.cos, -other.sin][x] [y_new] = [other.sin,
     * other.cos][y]
     *
     * <p>
     * For example, rotating a Translation2d of {2, 0} by 90 degrees will return a
     * Translation2d of {0, 2}.
     *
     * @param other The rotation to rotate the translation by.
     * @return The new rotated translation.
     */
    public Translation2d rotateBy(final Rotation2d rotation) {
        return new Translation2d(x_ * rotation.cos() - y_ * rotation.sin(), x_ * rotation.sin() + y_ * rotation.cos());
    }

    /**
     * Adds two translations in 2d space and returns the sum. This is similar to
     * vector addition.
     *
     * <p>
     * For example, Translation2d{1.0, 2.5} + Translation2d{2.0, 5.5} =
     * Translation2d{3.0, 8.0}
     *
     * @param other The translation to add.
     * @return The sum of the translations.
     */
    public Translation2d plus(Translation2d other) {
        return new Translation2d(x_ + other.x_, y_ + other.y_);
    }

    /**
     * Subtracts the other translation from the other translation and returns the
     * difference.
     *
     * <p>
     * For example, Translation2d{5.0, 4.0} - Translation2d{1.0, 2.0} =
     * Translation2d{4.0, 2.0}
     *
     * @param other The translation to subtract.
     * @return The difference between the two translations.
     */
    public Translation2d minus(Translation2d other) {
        return new Translation2d(x_ - other.x_, y_ - other.y_);
    }

    /**
     * Returns the inverse of the current translation. This is equivalent to
     * rotating by 180 degrees, flipping the point over both axes, or simply
     * negating both components of the translation.
     *
     * @return The inverse of the current translation.
     */
    public Translation2d unaryMinus() {
        return new Translation2d(-x_, -y_);
    }

    public Rotation2d direction() {
        return new Rotation2d(x_, y_, true);
    }

    /**
     * The inverse simply means a Translation2d that "undoes" this object.
     *
     * @return Translation by -x and -y.
     */
    public Translation2d inverse() {
        return new Translation2d(-x_, -y_);
    }

    @Override
    public Translation2d interpolate(final Translation2d other, double x) {
        if (x <= 0) {
            return new Translation2d(this);
        } else if (x >= 1) {
            return new Translation2d(other);
        }
        return extrapolate(other, x);
    }

    public Translation2d extrapolate(final Translation2d other, double x) {
        return new Translation2d(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    }

    public Translation2d scale(double s) {
        return new Translation2d(x_ * s, y_ * s);
    }

    public boolean epsilonEquals(final Translation2d other, double epsilon) {
        return Util.epsilonEquals(x(), other.x(), epsilon) && Util.epsilonEquals(y(), other.y(), epsilon);
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(x_) + "," + fmt.format(y_) + ")";
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(x_) + "," + fmt.format(y_);
    }

    @Override
    public int getNumFields(){
        return 2;
    }

    public static double dot(final Translation2d a, final Translation2d b) {
        return a.x_ * b.x_ + a.y_ * b.y_;
    }

    public static Rotation2d getAngle(final Translation2d a, final Translation2d b) {
        double cos_angle = dot(a, b) / (a.norm() * b.norm());
        if (Double.isNaN(cos_angle)) {
            return new Rotation2d();
        }
        return Rotation2d.fromRadians(Math.acos(Math.min(1.0, Math.max(cos_angle, -1.0))));
    }

    public static double cross(final Translation2d a, final Translation2d b) {
        return a.x_ * b.y_ - a.y_ * b.x_;
    }

    @Override
    public double distance(final Translation2d other) {
        return inverse().translateBy(other).norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Translation2d))
            return false;
        return distance((Translation2d) other) < Util.kEpsilon;
    }

    @Override
    public Translation2d getTranslation() {
        return this;
    }
}

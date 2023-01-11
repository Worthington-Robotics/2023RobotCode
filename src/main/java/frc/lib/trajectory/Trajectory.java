package frc.lib.trajectory;

import frc.lib.geometry.Pose2d;
import frc.lib.util.CSVWritable;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Represents a time-parameterized trajectory. The trajectory contains of
 * various States that represent the pose, curvature, time elapsed, velocity,
 * and acceleration at that point.
 */
public class Trajectory implements  CSVWritable {
    private final double m_totalTimeSeconds;
    private final List<TimedState> m_states;

    /**
     * Constructs a trajectory from a vector of states.
     *
     * @param states A list of states.
     */
    public Trajectory(final List<TimedState> states) {
        m_states = states;
        m_totalTimeSeconds = m_states.get(m_states.size() - 1).time;
    }

    /**
     * Returns the initial pose of the trajectory.
     *
     * @return The initial pose of the trajectory.
     */
    public Pose2d getInitialPose() {
        return sample(0).pose;
    }

    /**
     * Returns the overall duration of the trajectory.
     *
     * @return The duration of the trajectory.
     */
    public double getTotalTimeSeconds() {
        return m_totalTimeSeconds;
    }

    /**
     * Return the states of the trajectory.
     *
     * @return The states of the trajectory.
     */
    public List<TimedState> getStates() {
        return m_states;
    }

    /**
     * Sample the trajectory at a point in time.
     *
     * @param timeSeconds The point in time since the beginning of the trajectory to
     *                    sample.
     * @return The state at that point in time.
     */
    public TimedState sample(double timeSeconds) {
        if (timeSeconds <= m_states.get(0).time) {
            return m_states.get(0);
        }
        if (timeSeconds >= m_totalTimeSeconds) {
            return m_states.get(m_states.size() - 1);
        }

        // To get the element that we want, we will use a binary search algorithm
        // instead of iterating over a for-loop. A binary search is O(std::log(n))
        // whereas searching using a loop is O(n).

        // This starts at 1 because we use the previous state later on for
        // interpolation.
        int low = 1;
        int high = m_states.size() - 1;

        while (low != high) {
            int mid = (low + high) / 2;
            if (m_states.get(mid).time < timeSeconds) {
                // This index and everything under it are less than the requested
                // timestamp. Therefore, we can discard them.
                low = mid + 1;
            } else {
                // t is at least as large as the element at this index. This means that
                // anything after it cannot be what we are looking for.
                high = mid;
            }
        }

        // High and Low should be the same.

        // The sample's timestamp is now greater than or equal to the requested
        // timestamp. If it is greater, we need to interpolate between the
        // previous state and the current state to get the exact state that we
        // want.
        final TimedState sample = m_states.get(low);
        final TimedState prevSample = m_states.get(low - 1);

        // If the difference in states is negligible, then we are spot on!
        if (Math.abs(sample.time - prevSample.time) < 1E-9) {
            return sample;
        }
        // Interpolate between the two states for the state that we want.
        return prevSample.interpolate(sample, (timeSeconds - prevSample.time) / (sample.time - prevSample.time));
    }

    /**
     * Transforms all poses in the trajectory by the given transform. This is useful
     * for converting a robot-relative trajectory into a field-relative trajectory.
     * This works with respect to the first pose in the trajectory.
     *
     * @param transform The transform to transform the trajectory by.
     * @return The transformed trajectory.
     */
    public Trajectory transformBy(Pose2d transform) {
        TimedState firstState = m_states.get(0);
        Pose2d firstPose = firstState.pose;

        // Calculate the transformed first pose.
        var newFirstPose = firstPose.transformBy(transform);
        List<TimedState> newStates = new ArrayList<>();

        newStates.add(new TimedState(firstState.time, firstState.velocity, firstState.acceleration, newFirstPose,
                firstState.curvature));

        for (int i = 1; i < m_states.size(); i++) {
            var state = m_states.get(i);
            // We are transforming relative to the coordinate frame of the new initial pose.
            newStates.add(new TimedState(state.time, state.velocity, state.acceleration,
                    newFirstPose.plus(state.pose.minus(firstPose)), state.curvature));
        }

        return new Trajectory(newStates);
    }

    /**
     * Transforms all poses in the trajectory so that they are relative to the given
     * pose. This is useful for converting a field-relative trajectory into a
     * robot-relative trajectory.
     *
     * @param pose The pose that is the origin of the coordinate frame that the
     *             current trajectory will be transformed into.
     * @return The transformed trajectory.
     */
    public Trajectory relativeTo(Pose2d pose) {
        return new Trajectory(m_states.stream()
                .map(state -> new TimedState(state.time, state.velocity,
                        state.acceleration, state.pose.relativeTo(pose),
                        state.curvature))
                .collect(Collectors.toList()));
    }

    @Override
    public String toCSV() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < m_states.size(); ++i) {
            builder.append(i);
            builder.append(",");
            builder.append(m_states.get(i).toCSV());
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }

    @Override
    public int getNumFields() {
        return sample(0).getNumFields();
    }

    @Override
    public String toString() {
        String stateList = m_states.stream().map(TimedState::toString).collect(Collectors.joining(", \n"));
        return String.format("Trajectory - Seconds: %.2f, States:\n%s", m_totalTimeSeconds, stateList);
    }
}

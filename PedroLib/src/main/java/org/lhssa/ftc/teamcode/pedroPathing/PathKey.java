package org.lhssa.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

/**
 * Key object used by caching in TeamPaths.
 */
public class PathKey {
    private final Pose startPose;
    private final Pose endPose;
    private final HeadingInterpolationType headingInterpolationType;

    public PathKey(Pose startPose, Pose endPose, HeadingInterpolationType headingInterpolationType) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.headingInterpolationType = headingInterpolationType;
    }

    public Pose getStartPose() {
        return this.startPose;
    }

    public Pose getEndPose() {
        return this.endPose;
    }

    public HeadingInterpolationType getHeadingInterpolationType() {
        return this.headingInterpolationType;
    }

    /**
     * @param o The other PathKey
     * @return true if the passed PathKey has the same start Pose, end Pose, and heading
     * interpolation values as this PathKey.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof PathKey)) return false;
        PathKey other = (PathKey) o;
        return posesEqual(this.getStartPose(), other.getStartPose()) &&
                posesEqual(this.getEndPose(), other.getEndPose()) &&
                getHeadingInterpolationType() == other.getHeadingInterpolationType();
    }

    @Override
    public int hashCode() {
        return 31 * getStartPose().hashCode() + getEndPose().hashCode() + getHeadingInterpolationType().hashCode();
    }

    private boolean posesEqual(Pose pose1, Pose pose2) {
        return (pose1.getX() == pose2.getX() &&
                pose1.getY() == pose2.getY() &&
                pose1.getHeading() == pose2.getHeading());
    }
}

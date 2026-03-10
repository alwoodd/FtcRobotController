package org.lhssa.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;

import java.util.HashMap;
import java.util.Map;

/**
 * This class creates and returns Paths created from passed Poses.
 */
public class PedroPather {
    private final AllianceColor definedPoseColor = AllianceColor.RED;
    private final boolean mustFlip;

    private final Map<PathKey, Path> cache;

    public PedroPather(AllianceColor canonicalColor, AllianceColor allianceColor) {
        this.mustFlip = allianceColor != canonicalColor;
        this.cache = new HashMap<>();
    }

    /**
     * Create BezierLine Path using the passed startPose, endPose, and headingInterpolationType.
     * The headings come from the start and end Pose headings.
     * Paths are cached, so it is ok to call this method repeatedly.
     * @param startPose start Pose of BezierLine
     * @param endPose end Pose of BezierLine
     * @param headingInterpolationType LINEAR, TANGENT, CONSTANT
     * @return Path
     */
    public Path pathBetween(Pose startPose, Pose endPose, HeadingInterpolationType headingInterpolationType) {
        PathKey k = new PathKey(startPose, endPose, headingInterpolationType);
        if (cache.containsKey(k)) {
            return cache.get(k);
        }
        else {
            Path newPath = createPath(startPose, endPose);
            setHeadingInterpolation(newPath, headingInterpolationType);
            cache.put(k, newPath);
            return newPath;
        }
    }

    /**
     * Create BezierLine Path using the passed startPose and endPose. HeadingInterpolationType is
     * LINEAR.
     * Paths are cached, so it is ok to call this method repeatedly.
     * @param startPose start Pose of BezierLine
     * @param endPose end Pose of BezierLine
     * @return Path
     */
    public Path pathBetween(Pose startPose, Pose endPose) {
        return this.pathBetween(startPose, endPose, HeadingInterpolationType.LINEAR);
    }

    /**
     * Ensures that the passed pose is correct for the instantiated AllianceColor.
     * @param pose Pose to normalize
     * @return Pose
     */
    public Pose normalizePose(Pose pose) {
        Pose returnedPose = pose;

        if (mustFlip) {
            returnedPose = flipPose(pose);
        }

        return returnedPose;
    }

    /**
     * Create a BezierLine Path using the passed start and end Poses.
     * The passed Poses are flipped as needed.
     * @param startPose start Pose of BezierLine
     * @param endPose end Pose of BezierLine
     * @return Path
     */
    private Path createPath(Pose startPose, Pose endPose) {
        Pose workingStartPose;
        Pose workingEndPose;

        if (mustFlip) {
            workingStartPose = flipPose(startPose);
            workingEndPose = flipPose(endPose);
        }
        else {
            workingStartPose = new Pose(startPose.getX(), startPose.getY(), startPose.getHeading());
            workingEndPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
        }

        return new Path(new BezierLine(workingStartPose, workingEndPose));
    }

    /**
     * Call appropriate setXXXHeadingInterpolation based on passed headingInterpolationType.
     * @param path Path to set heading interpolation on
     * @param headingInterpolationType drives which setXXXHeadingInterpolation() to call
     */
    private void setHeadingInterpolation(Path path, HeadingInterpolationType headingInterpolationType) {
        switch (headingInterpolationType) {
            case LINEAR:
                path.setLinearHeadingInterpolation(path.getFirstControlPoint().getHeading(),
                        path.getLastControlPoint().getHeading());
                break;
            case TANGENT:
                path.setTangentHeadingInterpolation();
                break;
            case CONSTANT:
                path.setConstantHeadingInterpolation(path.getLastControlPoint().getHeading());
                break;
        }

    }

    private Pose flipPose(Pose oldPose) {
        return new Pose(
            144 - oldPose.getX(),
            oldPose.getY(),
            MathFunctions.normalizeAngle(Math.PI - oldPose.getHeading()));
    }
}

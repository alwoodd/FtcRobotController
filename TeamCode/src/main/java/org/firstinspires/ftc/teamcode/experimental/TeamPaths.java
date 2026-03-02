package org.firstinspires.ftc.teamcode.experimental;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.AllianceColor;

import java.util.HashMap;
import java.util.Map;

/**
 * Define all the team's Poses here. The Poses should be either for RED or BLUE, and the
 * definedPoseColor set accordingly.
 * This class can also create and return Paths created from passed Poses (which are normally
 * Poses defined in this class).
 */
public class TeamPaths {
    private final AllianceColor definedPoseColor = AllianceColor.RED;
    private final boolean mustFlip;

    private final Map<PathKey, Path> cache;

    public final Pose frontWallStartingPose = new Pose(85.0, 8.5, Math.toRadians(90));
    public final Pose backWallStartingPose = new Pose(121.3, 125.3, Math.toRadians(-142));
    public final Pose backGoalShootPose = new Pose(108.4, 120.3, Math.toRadians(28));
    public final Pose frontGoalShootPose = new Pose(85, 8.5, Math.toRadians(70));

    public final Pose startAudienceBallPickupPose = new Pose(108.7, 35.5, Math.toRadians(180));
    public final Pose endAudienceBallPickupPose = new Pose(119.9, 35.5, Math.toRadians(180));

    public final Pose startMiddleBallPickupPose = new Pose(108.7, 59.7, Math.toRadians(180));
    public final Pose endMiddleBallPickupPose = new Pose(119.9, 59.7, Math.toRadians(180));

    public final Pose startGoalBallPickupPose = new Pose(108.7, 83.5, Math.toRadians(180));
    public final Pose endGoalBallPickupPose = new Pose(119.9, 83.5, Math.toRadians(180));

    public final Pose parkPose = new Pose(38.8, 33.2, Math.toRadians(180));

    public TeamPaths(AllianceColor allianceColor) {
        this.mustFlip = allianceColor != definedPoseColor;
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

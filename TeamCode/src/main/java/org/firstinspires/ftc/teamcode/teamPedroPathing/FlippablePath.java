package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A FlippablePath extends Paths with the ability to flipRightToLeft, and to reverse.
 * WARNING: Does not support flipping Paths that have called reverseHeadingInterpolation().
 */
public class FlippablePath extends Path {
    private final HeadingInterpolationType headingInterpolationType;

    private enum HeadingInterpolationType {
        TANGENT,
        CONSTANT,
        LINEAR
    }

    private boolean isReversed = false;

    /**
     * Instantiate a FlippablePath and set it to TangentHeadingInterpolation.
     * @param curve Curve which is either a BezierLine or BezierCurve.
     */
    public FlippablePath(Curve curve) {
        super(curve);
        this.headingInterpolationType = HeadingInterpolationType.TANGENT;
        this.setTangentHeadingInterpolation();
    }
    /**
     * Instantiate a FlippablePath and set it to ConstantHeadingInterpolation.
     * @param curve Curve which is either a BezierLine or BezierCurve.
     * @param endHeading The interpolation's heading.
     */
    public FlippablePath(Curve curve, double endHeading) {
        super(curve);
        this.headingInterpolationType = HeadingInterpolationType.CONSTANT;
        this.setConstantHeadingInterpolation(endHeading);
    }

    /**
     * Instantiate a FlippablePath and set it to LinearHeadingInterpolation.
     * @param curve Curve which is either a BezierLine or BezierCurve.
     * @param startHeading The interpolation's start heading.
     * @param endHeading The interpolation's end heading.
     */
    public FlippablePath(Curve curve, double startHeading, double endHeading) {
        super(curve);
        this.headingInterpolationType = HeadingInterpolationType.LINEAR;
        this.setLinearHeadingInterpolation(startHeading, endHeading);
    }

    /**
     * Convenience method that explicitly creates a FlippablePath with TangentHeadingInterpolation.
     * @param curve Curve which is either a BezierLine or BezierCurve.
     * @return FlippablePath
     */
    public static FlippablePath tangentHeadingPath(Curve curve) {
        return new FlippablePath(curve);
    }
    /**
     * Convenience method that explicitly creates a FlippablePath with ConstantHeadingInterpolation.
     * @param curve Curve which is either a BezierLine or BezierCurve.
     * @param endHeading The interpolation's heading.
     * @return FlippablePath
     */
    public static FlippablePath constantHeadingPath(Curve curve, double endHeading) {
        return new FlippablePath(curve, endHeading);
    }
    /**
     * Convenience method that explicitly creates a FlippablePath with LinearHeadingInterpolation.
     * @param curve Curve which is either a BezierLine or BezierCurve.
     * @param startHeading The interpolation's start heading.
     * @param endHeading The interpolation's end heading.
     * @return FlippablePath
     */
    public static FlippablePath linearHeadingPath(Curve curve, double startHeading, double endHeading) {
        return new FlippablePath(curve, startHeading, endHeading);
    }

    /**
     * Create a new FlippablePath that is flipped Right to Left, or Left to Right.
     * @return FlippablePath
     */
    public FlippablePath flipPath() {
        ArrayList<Pose> newPoses = new ArrayList<>();

        for (Pose oldPose : this.getControlPoints()) {
            newPoses.add(new Pose(
                144 - oldPose.getX(),
                oldPose.getY(),
                MathFunctions.normalizeAngle(Math.PI - oldPose.getHeading()))
            );
        }

        return createFlippablePathFrom(this.headingInterpolationType, createCurveFrom(newPoses));
    }

    /**
     * Create a new FlippablePath that has its controlPoints (that is, its Poses)
     * in reverse order. Useful for making a FlippablePath that simply reverses
     * direction.
     * @return FlippablePath
     * @apiNote Why not just use Curve.getReversed()? It doesn't reverse the headings as expected.
     */
    public FlippablePath reverse() {
        Curve curve = this.getCurve();
        List<Pose> controlPoints = curve.getControlPoints();
        Collections.reverse(controlPoints);
        FlippablePath fPath = createFlippablePathFrom(this.headingInterpolationType, createCurveFrom(controlPoints));
        this.isReversed = !this.isReversed;
        return fPath;
    }

    /**
     * Create a Curve that is either a BezierLine or BezierCurve,
     * depending on the number of Poses passed.
     * @param  poses List of Poses from which to create a Curve.
     * @return Curve
     */
    private Curve createCurveFrom(List<Pose> poses) {
        Curve newCurve;

        if (poses.size() < 3) {
            //The first of the two Poses is assumed to be the start Pose. The second, the end Pose.
            newCurve = new BezierLine(poses.get(0), poses.get(1));
        }
        else {
            newCurve = new BezierCurve(poses);
        }

        return newCurve;
    }

    /**
     * Using passed headingInterpolationType, call the expected constructor, and return the instance.
     * @param headingInterpolationType determines which constructor to call.
     * @param newCurve the Curve for the new instance, and from which any required headings come from.
     * @return FlippablePath
     */
    private FlippablePath createFlippablePathFrom(HeadingInterpolationType headingInterpolationType, Curve newCurve) {
        FlippablePath newFlippablePath = null;

        switch (headingInterpolationType) {
            case TANGENT:
                newFlippablePath = new FlippablePath(newCurve);
                break;
            case CONSTANT:
                double heading = (isReversed) ? newCurve.getFirstControlPoint().getHeading() :
                        newCurve.getLastControlPoint().getHeading();
                newFlippablePath = new FlippablePath(newCurve, heading);
                break;
            case LINEAR:
                newFlippablePath = new FlippablePath(newCurve, newCurve.getFirstControlPoint().getHeading(),
                        newCurve.getLastControlPoint().getHeading());
        }

        return newFlippablePath;
    }
}

package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;

import java.util.ArrayList;

public class PedroPathConverter {
    private final Follower follower;

    public PedroPathConverter(Follower follower) {
        this.follower = follower;
    }

    public PathChain convertBluePathChainToRed(PathChain pathChain) {
        Path path;
        ArrayList<Pose> outPoses;
        PathBuilder builder = this.follower.pathBuilder();
        double maxXY = 144.0;
        double adjustedX;
        double adjustedY;
        BezierCurve bezierCurve;

        //For each Path in the passed pathChain
        for (int i = 0; i < pathChain.size(); i++) {
            path = pathChain.getPath(i);
            //For each Pose in the Path
            outPoses = new ArrayList<Pose>();
            for (Pose pose : path.getControlPoints()) {
                //Adjust the point's X,Y and create a new Point.
                adjustedX = maxXY - pose.getX();
                adjustedY = maxXY = pose.getY();
                outPoses.add(new Pose(adjustedX, adjustedY));
            }

            /*
             * Add a Path from using outPoses.
             * If the number of Poses < 3, construct a BezierLine.
             * Otherwise, construct a BezierCurve.
             * A BezierLine inherits from BezierCurve. Hence, a BezierLine is a BezierCurve.
             */
            if (outPoses.size() < 3) {
                //The first of the two Poses is assumed to be the start Pose. The second, the end Pose.
                bezierCurve = new BezierLine(outPoses.get(0), outPoses.get(1));
            }
            else {
                bezierCurve = new BezierCurve(outPoses);
            }
            builder.addPath(bezierCurve);
        }

        //return a new PathChain
        return builder.build();
    }
}

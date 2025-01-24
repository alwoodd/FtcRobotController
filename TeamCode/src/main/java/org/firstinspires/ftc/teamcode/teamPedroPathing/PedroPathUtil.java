package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.ArrayList;

public class PedroPathUtil {
    private final LinearOpMode opMode;
    private final Follower follower;

    public PedroPathUtil (LinearOpMode opMode, Follower follower) {
        this.opMode = opMode;
        this.follower = follower;
    }

    public void followPathChain(PathChain pathChain) {
        follower.followPath(pathChain);

        while (opMode.opModeIsActive() && !follower.isBusy()){
            follower.update();
        }
    }

    public static PathChain convertBluePathChainToRed(PathChain pathChain) {
        Path path;
        ArrayList<Point> outPoints;
        PathBuilder builder = new PathBuilder();
        double maxXY = 144.0;
        double adjustedX;
        double adjustedY;

        //For each Path in the passed pathChain
        for (int i = 0; i < pathChain.size(); i++) {
            path = pathChain.getPath(i);
            //For each Point in the Path
            outPoints = new ArrayList<Point>();
            for (Point point : path.getControlPoints()) {
                //Adjust the point's X,Y and create a new Point.
                adjustedX = maxXY - point.getX();
                adjustedY = maxXY = point.getY();
                outPoints.add(new Point(adjustedX, adjustedY, Point.CARTESIAN));
            }

            //Add a Path from a new BezierCurve using outPoints.
            builder.addPath(new BezierCurve(outPoints));
        }

        //return a new PathChain
        return builder.build();
    }
}

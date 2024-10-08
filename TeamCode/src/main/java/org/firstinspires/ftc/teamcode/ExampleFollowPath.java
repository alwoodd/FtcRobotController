package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.ArrayList;

public class ExampleFollowPath extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = new Follower(hardwareMap);

        //Define Point coordinates from which our Bezier curve will be calculated.
        ArrayList<Point> pointList = new ArrayList<Point>();
        pointList.add(new Point(-55, 0, Point.CARTESIAN));
        pointList.add(new Point(-50, -50, Point.CARTESIAN));
        pointList.add(new Point(-40, -80, Point.CARTESIAN));
        pointList.add(new Point(0, -20, Point.CARTESIAN));

        //Create the Bezier curve, then use them for the Path to follow.
        BezierCurve curve = new BezierCurve(pointList);
        Path path = new Path(curve);
        follower.followPath(path);

        waitForStart();

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }
}

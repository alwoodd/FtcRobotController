package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;

public class RedPedroPathsFrontWall implements AutonomousPedroPathsFrontWall {
    private final Follower follower;

    private PathChain pathFromWallToLaunchZone;
    private PathChain pathFromLaunchZoneToBallPickup;
    private PathChain pathFromBallPickupToLaunchZone;
    private final Pose startingPose = new Pose(85.0, 8.5, Math.toRadians(90));

    private final Pose goalShootPose = new Pose(108.378, 120.308, Math.toRadians(4));
    private final Pose ballPickupPose = new Pose(108.7, 84, Math.toRadians(180));

    public RedPedroPathsFrontWall(Follower follower) {
        this.follower = follower;
        initPaths();
    }

    private void initPaths() {
        this.pathFromWallToLaunchZone = buildPathFromWallToLaunchZone();
        this.pathFromLaunchZoneToBallPickup = buildPathFromLaunchZoneToBallPickup();
        this.pathFromBallPickupToLaunchZone = buildPathFromBallPickupToLaunchZone();
    }

    @Override
    public Pose startingPose() {
        return this.startingPose;
    }

    private PathChain buildPathFromWallToLaunchZone() {
        PathBuilder builder = follower.pathBuilder();
        builder
            .addPath(
                // Line 1
                new BezierLine(
                startingPose,
                new Pose(85, 95, Math.toRadians(90)))
                )
            .setTangentHeadingInterpolation()
            .addPath(
                // Line 2
                new BezierCurve(
                new Pose(85, 95, Math.toRadians(90)),
                new Pose(85.3, 116.6),
                new Pose(108.378, 120.308, Math.toRadians(4))
                )
            )
            .setTangentHeadingInterpolation();
        return builder.build();
    }

    private PathChain buildPathFromLaunchZoneToBallPickup() {
        PathBuilder builder = follower.pathBuilder();
        builder
            .addPath(new BezierLine(goalShootPose, ballPickupPose))
            .setLinearHeadingInterpolation(Math.toRadians(9), Math.toRadians(180));

        return builder.build();
    }

    private PathChain buildPathFromBallPickupToLaunchZone() {
        Path thePath = pathFromLaunchZoneToBallPickup.firstPath(); //and only path
        BezierLine bezierLine = (BezierLine) thePath.getCurve();
        ArrayList<Pose> controlPoints = bezierLine.getControlPoints();
        Pose reverseStartPose = controlPoints.get(1);
        Pose reverseEndPose = controlPoints.get(0);
        Path reversePath = new Path(new BezierLine(reverseStartPose, reverseEndPose));
        reversePath.setLinearHeadingInterpolation(reverseStartPose.getHeading(), reverseEndPose.getHeading());
        return new PathChain(reversePath);
    }

    @Override
    public PathChain pathFromWallToLaunchZone() {
        return pathFromWallToLaunchZone;
    }

    @Override
    public PathChain pathFromLaunchZoneToBallPickup() {
        return pathFromLaunchZoneToBallPickup;
    }

    @Override
    public PathChain pathFromBallPickupToLaunchZone() {
        return pathFromBallPickupToLaunchZone;
    }
}

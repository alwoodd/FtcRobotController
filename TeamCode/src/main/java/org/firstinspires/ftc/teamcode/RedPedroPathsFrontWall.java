package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.teamPedroPathing.FlippablePath;

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

    /**
     * These Paths have tangent headings.
     * @return PathChain
     */
    private PathChain buildPathFromWallToLaunchZone() {
        PathBuilder builder = follower.pathBuilder();
        builder
            .addPath(
                FlippablePath.tangentHeadingPath(
                    new BezierLine(
                        startingPose,
                        new Pose(85, 95, Math.toRadians(90))
                    )
                )
            )
            .addPath(
                FlippablePath.tangentHeadingPath(
                    new BezierCurve(
                        new Pose(85, 95, Math.toRadians(90)),
                        new Pose(85.3, 116.6),
                        new Pose(108.378, 120.308, Math.toRadians(4))
                    )
                )
            );

        return builder.build();
    }

    /**
     * This Path has a linear heading, with a start heading of goalShootPose,
     * and the end heading of ballPickupPose.
     * @return PathChain
     */
    private PathChain buildPathFromLaunchZoneToBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(goalShootPose, ballPickupPose),
                goalShootPose.getHeading(), ballPickupPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    /**
     *
     * @return PathChain
     */
    private PathChain buildPathFromBallPickupToLaunchZone() {
        FlippablePath thePath = (FlippablePath) pathFromLaunchZoneToBallPickup.firstPath(); //and only path
        return new PathChain(thePath.reverse());
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

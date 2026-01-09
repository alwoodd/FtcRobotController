package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.teamPedroPathing.FlippablePath;

public class RedPedroPathsFrontWall implements AutonomousPedroPathsFrontWall {
    private final Follower follower;

    private PathChain pathFromWallToLaunchZone;
    private PathChain pathFromLaunchZoneToStartBallPickup;
    private PathChain pathFromStartBallPickupToEndBallPickup;
    private PathChain pathFromEndBallPickupToLaunchZone;
    //private PathChain pathFromStartBallPickupToLaunchZone;
    private PathChain pathFromLaunchZoneToPark;

    private final Pose startingPose = new Pose(85.0, 8.5, Math.toRadians(90));
    private final Pose goalShootPose = new Pose(108.4, 120.3, Math.toRadians(28));
    private final Pose startBallPickupPose = new Pose(108.7, 83.5, Math.toRadians(180));
    private final Pose endBallPickupPose = new Pose(119.9, 83.5, Math.toRadians(0));

    public RedPedroPathsFrontWall(Follower follower) {
        this.follower = follower;
        initPaths();
    }

    private void initPaths() {
        this.pathFromWallToLaunchZone = buildPathFromWallToLaunchZone();
        this.pathFromLaunchZoneToStartBallPickup = buildPathFromLaunchZoneToStartBallPickup();
        this.pathFromStartBallPickupToEndBallPickup = buildPathFromStartToEndBallPickup();
        this.pathFromEndBallPickupToLaunchZone = buildPathFromEndPickupToLaunchZone();
        //this.pathFromStartBallPickupToLaunchZone = buildPathFromStartBallPickupToLaunchZone();
        this.pathFromLaunchZoneToPark = buildPathFromLaunchZoneToPark();
    }

    @Override
    public Pose startingPose() {
        return this.startingPose;
    }

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
                        new Pose(91.366, 111.208),
                        goalShootPose
                    )
                )
            );

        return builder.build();
    }

    private PathChain buildPathFromLaunchZoneToStartBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(goalShootPose, startBallPickupPose),
                goalShootPose.getHeading(), startBallPickupPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildPathFromStartToEndBallPickup() {
        FlippablePath fPath = FlippablePath.constantHeadingPath(new BezierLine(startBallPickupPose, endBallPickupPose), startBallPickupPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildPathFromEndPickupToLaunchZone() {
        FlippablePath fPath = (FlippablePath) pathFromStartBallPickupToEndBallPickup.firstPath();
        FlippablePath pathFromEndToStartPickup = fPath.reverse();

        fPath = (FlippablePath) pathFromLaunchZoneToStartBallPickup.firstPath();
        FlippablePath pathFromStartPickupToLaunch = fPath.reverse();

        return follower.pathBuilder()
            .addPath(pathFromEndToStartPickup)
            .addPath(pathFromStartPickupToLaunch)
            .build();
    }

    private PathChain buildPathFromLaunchZoneToPark() {
        return pathFromLaunchZoneToStartBallPickup;
    }

    @Override
    public PathChain pathFromWallToLaunchZone() {
        return pathFromWallToLaunchZone;
    }

    @Override
    public PathChain pathFromLaunchZoneToStartBallPickup() {
        return pathFromLaunchZoneToStartBallPickup;
    }

    //SLOW
    @Override
    public PathChain pathFromStartBallPickupToEndBallPickup() {
        return pathFromStartBallPickupToEndBallPickup;
    }

    @Override
    public PathChain pathFromEndBallPickupToLaunchZone() {
        return pathFromEndBallPickupToLaunchZone;
    }

/*
    @Override
    public PathChain pathFromStartBallPickupToLaunchZone() {
        return pathFromStartBallPickupToLaunchZone;
    }
*/

    @Override
    public PathChain pathFromLaunchZoneToPark() {
        return pathFromLaunchZoneToPark;
    }
}

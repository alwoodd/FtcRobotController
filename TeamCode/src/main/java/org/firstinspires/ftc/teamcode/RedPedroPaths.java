package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.teamPedroPathing.FlippablePath;

public class RedPedroPaths implements AutonomousPaths {
    private final Follower follower;

    private final Pose frontWallStartingPose = new Pose(85.0, 8.5, Math.toRadians(90));
    private final Pose backWallStartingPose = new Pose();
    private final Pose goalShootPose = new Pose(108.4, 120.3, Math.toRadians(28));

    private final Pose startAudienceBallPickupPose = new Pose(108.7, 35.5, Math.toRadians(180));
    private final Pose endAudienceBallPickupPose = new Pose(119.9, 35.5, Math.toRadians(180));

    private final Pose startMiddleBallPickupPose = new Pose(108.7, 59.7, Math.toRadians(180));
    private final Pose endMiddleBallPickupPose = new Pose(119.9, 59.7, Math.toRadians(180));

    private final Pose startGoalBallPickupPose = new Pose(108.7, 83.5, Math.toRadians(180));
    private final Pose endGoalBallPickupPose = new Pose(119.9, 83.5, Math.toRadians(180));

    private final Pose parkPose = new Pose(38.8, 33.2, Math.toRadians(180));

    private PathChain pathFromFrontWallToLaunchZone;
    private PathChain pathFromBackWallToLaunchZone;

    private PathChain pathFromLaunchZoneToAudienceSideBallPickup;
    private PathChain pathFromLaunchZoneToMiddleSideBallPickup;
    private PathChain pathFromLaunchZoneToGoalSideBallPickup;

    private PathChain pathFromAudienceSideBallPickupToEndBallPickup;
    private PathChain pathFromMiddleSideBallPickupToEndBallPickup;
    private PathChain pathFromGoalSideBallPickupToEndBallPickup;

    private PathChain pathFromAudienceSideEndBallPickupToLaunchZone;
    private PathChain pathFromMiddleSideEndBallPickupToLaunchZone;
    private PathChain pathFromGoalSideEndBallPickupToLaunchZone;

    private PathChain pathFromLaunchZoneToAudienceSideLeave;
    private PathChain pathFromLaunchZoneToMiddleSideLeave;
    private PathChain pathFromLaunchZoneToGoalSideLeave;

    public RedPedroPaths(Follower follower) {
        this.follower = follower;
        initPaths();
    }

    private void initPaths() {
        this.pathFromFrontWallToLaunchZone = buildPathFromFrontWallToLaunchZone();
        this.pathFromBackWallToLaunchZone = buildPathFromBackWallToLaunchZone();

        this.pathFromLaunchZoneToAudienceSideBallPickup = buildPathFromLaunchZoneToAudienceSideBallPickup();
        this.pathFromLaunchZoneToMiddleSideBallPickup = buildPathFromLaunchZoneToMiddleSideBallPickup();
        this.pathFromLaunchZoneToGoalSideBallPickup = buildPathFromLaunchZoneToGoalSideBallPickup();

        this.pathFromAudienceSideBallPickupToEndBallPickup = buildPathFromAudienceSideBallPickupToEndBallPickup();
        this.pathFromMiddleSideBallPickupToEndBallPickup = buildPathFromMiddleSideBallPickupToEndBallPickup();
        this.pathFromGoalSideBallPickupToEndBallPickup = buildPathFromGoalSideBallPickupToEndBallPickup();

        this.pathFromAudienceSideEndBallPickupToLaunchZone = buildpathFromAudienceSideEndBallPickupToLaunchZone();
        this.pathFromMiddleSideEndBallPickupToLaunchZone = buildpathFromMiddleSideEndBallPickupToLaunchZone();
        this.pathFromGoalSideEndBallPickupToLaunchZone = buildPathFromGoalSideEndPickupToLaunchZone();

        this.pathFromLaunchZoneToAudienceSideLeave = buildPathFromLaunchZoneToAudienceSideLeave();
        this.pathFromLaunchZoneToMiddleSideLeave = buildPathFromLaunchZoneToMiddleSideLeave();
        this.pathFromLaunchZoneToGoalSideLeave = buildPathFromLaunchZoneToGoalSideLeave();
    }

    private PathChain buildPathFromFrontWallToLaunchZone() {
        PathBuilder builder = follower.pathBuilder();
        builder
            .addPath(
                FlippablePath.tangentHeadingPath(
                    new BezierLine(
                            frontWallStartingPose,
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

    private PathChain buildPathFromBackWallToLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(backWallStartingPose, goalShootPose),
            backWallStartingPose.getHeading(), goalShootPose. getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    private PathChain buildPathFromLaunchZoneToAudienceSideBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(goalShootPose, startAudienceBallPickupPose),
                goalShootPose.getHeading(), startAudienceBallPickupPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildPathFromLaunchZoneToMiddleSideBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(goalShootPose, startMiddleBallPickupPose),
                goalShootPose.getHeading(), startMiddleBallPickupPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    private PathChain buildPathFromLaunchZoneToGoalSideBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(goalShootPose, startGoalBallPickupPose),
                goalShootPose.getHeading(), startMiddleBallPickupPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    private PathChain buildPathFromAudienceSideBallPickupToEndBallPickup() {
        FlippablePath fPath = FlippablePath.constantHeadingPath(new BezierLine(startAudienceBallPickupPose, endAudienceBallPickupPose),
                startAudienceBallPickupPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildPathFromMiddleSideBallPickupToEndBallPickup() {
        FlippablePath fPath = FlippablePath.constantHeadingPath(new BezierLine(startMiddleBallPickupPose, endMiddleBallPickupPose),
                startMiddleBallPickupPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildPathFromGoalSideBallPickupToEndBallPickup() {
        FlippablePath fPath = FlippablePath.constantHeadingPath(new BezierLine(startGoalBallPickupPose, endGoalBallPickupPose),
                startGoalBallPickupPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildpathFromAudienceSideEndBallPickupToLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(endAudienceBallPickupPose, goalShootPose),
                endAudienceBallPickupPose.getHeading(), goalShootPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildpathFromMiddleSideEndBallPickupToLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(endMiddleBallPickupPose, goalShootPose),
                endMiddleBallPickupPose.getHeading(), goalShootPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    /*
     * This demonstrates using FlippablePath.reverse(). However, this is not the most efficient
     * way to get back to the launch zone.
     */
    private PathChain buildPathFromGoalSideEndPickupToLaunchZone() {
        FlippablePath fPath = (FlippablePath) pathFromGoalSideBallPickupToEndBallPickup.firstPath();
        FlippablePath pathFromEndToStartPickup = fPath.reverse();

        fPath = (FlippablePath) pathFromLaunchZoneToAudienceSideBallPickup.firstPath();
        FlippablePath pathFromStartPickupToLaunch = fPath.reverse();

        return follower.pathBuilder()
            .addPath(pathFromEndToStartPickup)
            .addPath(pathFromStartPickupToLaunch)
            .build();
    }

    private PathChain buildPathFromLaunchZoneToAudienceSideLeave() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(goalShootPose, startAudienceBallPickupPose),
                goalShootPose.getHeading(), startAudienceBallPickupPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildPathFromLaunchZoneToMiddleSideLeave() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(goalShootPose, startMiddleBallPickupPose),
                goalShootPose.getHeading(), startMiddleBallPickupPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    private PathChain buildPathFromLaunchZoneToGoalSideLeave() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(goalShootPose, startGoalBallPickupPose),
                goalShootPose.getHeading(), startGoalBallPickupPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }
    /***********************************************************************************/
    @Override
    public Pose frontWallstartingPose() {
        return this.frontWallStartingPose;
    }

    @Override
    public Pose backWallstartingPose() {
        return this.frontWallStartingPose;
    }

    @Override
    public Pose parkPose() {
        return this.parkPose;
    }

    @Override
    public PathChain pathFromFrontWallToLaunchZone() {
        return pathFromFrontWallToLaunchZone;
    }

    @Override
    public PathChain pathFromBackWallToLaunchZone() {
        return pathFromBackWallToLaunchZone;
    }    /***********************************************************************************/

    @Override
    public PathChain pathFromLaunchZoneToAudienceSideBallPickup() {
        return pathFromLaunchZoneToAudienceSideBallPickup;
    }

    @Override
    public PathChain pathFromLaunchZoneToMiddleSideBallPickup() {
        return pathFromLaunchZoneToMiddleSideBallPickup;
    }

    @Override
    public PathChain pathFromLaunchZoneToGoalSideBallPickup() {
        return pathFromLaunchZoneToGoalSideBallPickup;
    }
    /***********************************************************************************/

    @Override
    public PathChain pathFromAudienceSideBallPickupToEndBallPickup() {
        return pathFromAudienceSideBallPickupToEndBallPickup;
    }

    @Override
    public PathChain pathFromMiddleSideBallPickupToEndBallPickup() {
        return pathFromMiddleSideBallPickupToEndBallPickup;
    }

    @Override
    public PathChain pathFromGoalSideBallPickupToEndBallPickup() {
        return pathFromGoalSideBallPickupToEndBallPickup;
    }
    /***********************************************************************************/

    @Override
    public PathChain pathFromAudienceSideEndBallPickupToLaunchZone() {
        return pathFromAudienceSideEndBallPickupToLaunchZone;
    }

    @Override
    public PathChain pathFromMiddleSideEndBallPickupToLaunchZone() {
        return pathFromMiddleSideEndBallPickupToLaunchZone;
    }

    @Override
    public PathChain pathFromGoalSideEndBallPickupToLaunchZone() {
        return pathFromGoalSideEndBallPickupToLaunchZone;
    }
    /***********************************************************************************/

    @Override
    public PathChain pathFromLaunchZoneToAudienceSideLeave() {
        return pathFromLaunchZoneToAudienceSideLeave;
    }

    @Override
    public PathChain pathFromLaunchZoneToMiddleSideLeave() {
        return pathFromLaunchZoneToMiddleSideLeave;
    }

    @Override
    public PathChain pathFromLaunchZoneToGoalSideLeave() {
        return pathFromLaunchZoneToGoalSideLeave;
    }
}

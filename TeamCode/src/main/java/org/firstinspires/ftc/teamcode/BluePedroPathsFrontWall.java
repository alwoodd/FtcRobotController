package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathFlipper;

public class BluePedroPathsFrontWall implements AutonomousPathsFrontWall {
    private final AutonomousPathsFrontWall redPedroPaths;
    private final PedroPathFlipper pathFlipper;

    private PathChain pathFromWallToLaunchZone;

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

    public BluePedroPathsFrontWall(Follower follower, AutonomousPathsFrontWall redPedroPaths) {
        this.redPedroPaths = redPedroPaths;
        this.pathFlipper = new PedroPathFlipper(follower);
        initPaths();
    }

    private void initPaths() {
        this.pathFromWallToLaunchZone = buildPathFromWallToLaunchZone();

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

    private PathChain buildPathFromWallToLaunchZone() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromWallToLaunchZone());
    }

    private PathChain buildPathFromLaunchZoneToAudienceSideBallPickup() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromLaunchZoneToAudienceSideBallPickup());
    }

    private PathChain buildPathFromLaunchZoneToMiddleSideBallPickup() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromLaunchZoneToMiddleSideBallPickup());
    }

    private PathChain buildPathFromLaunchZoneToGoalSideBallPickup() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromLaunchZoneToGoalSideBallPickup());
    }

    private PathChain buildPathFromAudienceSideBallPickupToEndBallPickup() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromAudienceSideBallPickupToEndBallPickup());
    }

    private PathChain buildPathFromMiddleSideBallPickupToEndBallPickup() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromMiddleSideBallPickupToEndBallPickup());
    }

    private PathChain buildPathFromGoalSideBallPickupToEndBallPickup() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromGoalSideBallPickupToEndBallPickup());
    }

    private PathChain buildpathFromAudienceSideEndBallPickupToLaunchZone() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromAudienceSideEndBallPickupToLaunchZone());
    }

    private PathChain buildpathFromMiddleSideEndBallPickupToLaunchZone() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromMiddleSideEndBallPickupToLaunchZone());
    }

    private PathChain buildPathFromGoalSideEndPickupToLaunchZone() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromGoalSideEndBallPickupToLaunchZone());
    }

    private PathChain buildPathFromLaunchZoneToAudienceSideLeave() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromLaunchZoneToAudienceSideLeave());
    }

    private PathChain buildPathFromLaunchZoneToMiddleSideLeave() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromLaunchZoneToMiddleSideLeave());
    }

    private PathChain buildPathFromLaunchZoneToGoalSideLeave() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromLaunchZoneToGoalSideLeave());
    }
    /***********************************************************************************/
    @Override
    public Pose startingPose() {
        return this.pathFromWallToLaunchZone.firstPath().getFirstControlPoint();
    }

    @Override
    public PathChain pathFromWallToLaunchZone() {
        return pathFromWallToLaunchZone;
    }
    /***********************************************************************************/

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

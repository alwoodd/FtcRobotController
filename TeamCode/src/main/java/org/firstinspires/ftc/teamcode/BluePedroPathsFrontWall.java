package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathFlipper;

public class BluePedroPathsFrontWall implements AutonomousPedroPathsFrontWall {
    private final AutonomousPedroPathsFrontWall redPedroPaths;
    private final PedroPathFlipper pathFlipper;

    private PathChain pathFromWallToLaunchZone;
    private PathChain pathFromLaunchZoneToStartBallPickup;
    private PathChain pathFromStartBallPickupToEndBallPickup;
    private PathChain pathFromEndBallPickupToLaunchZone;
    //private PathChain pathFromStartBallPickupToLaunchZone;
    private PathChain pathFromLaunchZoneToPark;

    public BluePedroPathsFrontWall(Follower follower, AutonomousPedroPathsFrontWall redPedroPaths) {
        this.redPedroPaths = redPedroPaths;
        this.pathFlipper = new PedroPathFlipper(follower);
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
        return this.pathFromWallToLaunchZone.firstPath().getFirstControlPoint();
    }

    private PathChain buildPathFromWallToLaunchZone() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromWallToLaunchZone());
    }

    private PathChain buildPathFromLaunchZoneToStartBallPickup() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromLaunchZoneToStartBallPickup());
    }

    private PathChain buildPathFromStartToEndBallPickup() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromStartBallPickupToEndBallPickup());
    }

    private PathChain buildPathFromEndPickupToLaunchZone() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromEndBallPickupToLaunchZone());
    }

    private PathChain buildPathFromLaunchZoneToPark() {
        return pathFlipper.flipPathChain(redPedroPaths.pathFromLaunchZoneToPark());
    }

    @Override
    public PathChain pathFromWallToLaunchZone() {
        return pathFromWallToLaunchZone;
    }

    @Override
    public PathChain pathFromLaunchZoneToStartBallPickup() {
        return pathFromLaunchZoneToStartBallPickup;
    }

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

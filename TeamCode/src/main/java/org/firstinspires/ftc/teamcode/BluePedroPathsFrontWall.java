package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathFlipper;

public class BluePedroPathsFrontWall implements AutonomousPedroPathsFrontWall {
    private final AutonomousPedroPathsFrontWall redPedroPaths;
    private final PedroPathFlipper pathFlipper;

    private PathChain pathFromWallToLaunchZone;
    private PathChain pathFromLaunchZoneToBallPickup;
    private PathChain pathFromBallPickupToLaunchZone;

    public BluePedroPathsFrontWall(Follower follower, AutonomousPedroPathsFrontWall redPedroPaths) {
        this.redPedroPaths = redPedroPaths;
        this.pathFlipper = new PedroPathFlipper(follower);
        initPaths();
    }

    private void initPaths() {
        this.pathFromWallToLaunchZone = buildPathFromWallToLaunchZone();
        this.pathFromLaunchZoneToBallPickup = buildPathFromLaunchZoneToBallPickup();
        this.pathFromBallPickupToLaunchZone = buildPathFromBallPickupToLaunchZone();
    }

    @Override
    public Pose startingPose() {
        return this.pathFromWallToLaunchZone.firstPath().getFirstControlPoint();
    }

    private PathChain buildPathFromWallToLaunchZone() {
        return pathFlipper.flipRightToLeft(redPedroPaths.pathFromWallToLaunchZone());
    }

    private PathChain buildPathFromLaunchZoneToBallPickup() {
        return pathFlipper.flipRightToLeft(redPedroPaths.pathFromLaunchZoneToBallPickup());
    }

    private PathChain buildPathFromBallPickupToLaunchZone() {
        return pathFlipper.flipRightToLeft(redPedroPaths.pathFromBallPickupToLaunchZone());
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

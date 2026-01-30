package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public interface AutonomousPathsFrontWall {
    Pose startingPose();
    /***********************************************************************************/
    PathChain pathFromWallToLaunchZone();
    /***********************************************************************************/
    PathChain pathFromLaunchZoneToAudienceSideBallPickup();
    PathChain pathFromLaunchZoneToMiddleSideBallPickup();
    PathChain pathFromLaunchZoneToGoalSideBallPickup();
    /***********************************************************************************/
    PathChain pathFromAudienceSideBallPickupToEndBallPickup();
    PathChain pathFromMiddleSideBallPickupToEndBallPickup();
    PathChain pathFromGoalSideBallPickupToEndBallPickup();

    /***********************************************************************************/
    PathChain pathFromAudienceSideEndBallPickupToLaunchZone();
    PathChain pathFromMiddleSideEndBallPickupToLaunchZone();
    PathChain pathFromGoalSideEndBallPickupToLaunchZone();
    /***********************************************************************************/
    PathChain pathFromLaunchZoneToAudienceSideLeave();
    PathChain pathFromLaunchZoneToMiddleSideLeave();
    PathChain pathFromLaunchZoneToGoalSideLeave();
}

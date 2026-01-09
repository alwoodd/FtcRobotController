package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public interface AutonomousPedroPathsFrontWall {
    Pose startingPose();
    PathChain pathFromWallToLaunchZone();
    PathChain pathFromLaunchZoneToStartBallPickup();
    PathChain pathFromStartBallPickupToEndBallPickup();
    PathChain pathFromEndBallPickupToLaunchZone();
    //PathChain pathFromStartBallPickupToLaunchZone();
    PathChain pathFromLaunchZoneToPark();
}

package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.paths.PathChain;

public interface AutonomousPedroPaths {
    PathChain pathFromWallToLaunchZone();
    PathChain pathFromChamberToSpike();
    PathChain pathFromSpikeToNetZone();
}

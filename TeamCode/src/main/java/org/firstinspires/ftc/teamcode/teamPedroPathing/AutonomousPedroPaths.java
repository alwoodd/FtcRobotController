package org.firstinspires.ftc.teamcode.teamPedroPathing;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public interface AutonomousPedroPaths {
    PathChain pathFromWallToChamber();
    PathChain pathFromChamberToSpike();
    PathChain pathFromSpikeToNetZone();
}

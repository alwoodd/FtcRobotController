package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.teamPedroPathing.AutonomousPedroPaths;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathUtil;

public class RedPedroPathsCenterWall implements AutonomousPedroPaths {
    private final AutonomousPedroPaths bluePedroPaths;

    public RedPedroPathsCenterWall(AutonomousPedroPaths bluePedroPaths) {
        this.bluePedroPaths = bluePedroPaths;
    }

    @Override
    public PathChain pathFromWallToChamber() {
        return PedroPathUtil.convertBluePathChainToRed(bluePedroPaths.pathFromWallToChamber());
    }

    @Override
    public PathChain pathFromChamberToSpike() {
        return PedroPathUtil.convertBluePathChainToRed((bluePedroPaths.pathFromChamberToSpike()));
    }

    @Override
    public PathChain pathFromSpikeToNetZone() {
        return PedroPathUtil.convertBluePathChainToRed((bluePedroPaths.pathFromSpikeToNetZone()));
    }
}

package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.teamPedroPathing.AutonomousPedroPaths;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConverter;

public class RedPedroPathsFrontWall implements AutonomousPedroPaths {
    private final AutonomousPedroPaths bluePedroPaths;
    private final PedroPathConverter converter;

    private PathChain pathFromWallToLaunchZone;
    private PathChain pathFromChamberToSpike;
    private PathChain pathFromSpikeToNetZone;

    public RedPedroPathsFrontWall(Follower follower, AutonomousPedroPaths bluePedroPaths) {
        this.bluePedroPaths = bluePedroPaths;
        this.converter = new PedroPathConverter(follower);
        initPaths();
    }

    private void initPaths() {
        this.pathFromWallToLaunchZone = initPathFromWallToLaunchZone();
        this.pathFromChamberToSpike = initPathFromChamberToSpike();
        this.pathFromSpikeToNetZone = initPathFromSpikeToNetZone();
    }

    private PathChain initPathFromWallToLaunchZone() {
        return converter.convertBluePathChainToRed(bluePedroPaths.pathFromWallToLaunchZone());
    }

    private PathChain initPathFromChamberToSpike() {
        return converter.convertBluePathChainToRed((bluePedroPaths.pathFromChamberToSpike()));
    }

    private PathChain initPathFromSpikeToNetZone() {
        return converter.convertBluePathChainToRed((bluePedroPaths.pathFromSpikeToNetZone()));
    }

    @Override
    public PathChain pathFromWallToLaunchZone() {
        return pathFromWallToLaunchZone;
    }

    @Override
    public PathChain pathFromChamberToSpike() {
        return pathFromChamberToSpike;
    }

    @Override
    public PathChain pathFromSpikeToNetZone() {
        return pathFromSpikeToNetZone;
    }
}

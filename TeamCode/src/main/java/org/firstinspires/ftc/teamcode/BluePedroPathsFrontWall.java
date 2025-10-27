package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.teamPedroPathing.AutonomousPedroPaths;

public class BluePedroPathsFrontWall implements AutonomousPedroPaths {
    private final Follower follower;

    private PathChain pathFromWallToLaunchZone;
    private PathChain pathFromChamberToSpike;
    private PathChain pathFromSpikeToNetZone;

    public BluePedroPathsFrontWall(Follower follower) {
        this.follower = follower;
        initPaths();
    }

    private void initPaths() {
        this.pathFromWallToLaunchZone = initPathFromWallToLaunchZone();
        this.pathFromChamberToSpike = initPathFromChamberToSpike();
        this.pathFromSpikeToNetZone = initPathFromSpikeToNetZone();
    }

    private PathChain initPathFromWallToLaunchZone() {
        PathBuilder builder = this.follower.pathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                            new Pose(56.0, 8.0),
                            new Pose(73.596, 96.616))
                )
                .addPath(
                        // Line 2
                        new BezierCurve(
                            new Pose(73.596, 96.616),
                            new Pose(82.166, 118.292),
                            new Pose(108.378, 120.308)
                        )
                );
        return builder.build();
    }

    private PathChain initPathFromChamberToSpike() {
        return new PathChain();
    }

    private PathChain initPathFromSpikeToNetZone() {
        return new PathChain();
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

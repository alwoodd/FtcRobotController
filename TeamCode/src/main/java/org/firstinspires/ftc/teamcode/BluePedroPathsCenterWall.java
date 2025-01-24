package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.teamPedroPathing.AutonomousPedroPaths;

public class BluePedroPathsCenterWall implements AutonomousPedroPaths {
    @Override
    public PathChain pathFromWallToChamber() {
        PathBuilder builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.757, 84.983, Point.CARTESIAN),
                                new Point(26.544, 84.971, Point.CARTESIAN),
                                new Point(32.856, 72.720, Point.CARTESIAN)
                        )
                )
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(32.856, 72.720, Point.CARTESIAN),
                                new Point(37.496, 57.313, Point.CARTESIAN),
                                new Point(52.903, 59.912, Point.CARTESIAN)
                        )
                );
        return builder.build();
    }

    @Override
    public PathChain pathFromChamberToSpike() {
        return new PathChain();
    }

    @Override
    public PathChain pathFromSpikeToNetZone() {
        return new PathChain();
    }
}

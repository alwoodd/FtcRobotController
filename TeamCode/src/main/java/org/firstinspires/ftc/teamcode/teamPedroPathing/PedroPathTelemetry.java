package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PedroPathTelemetry {
    private final Telemetry telemetry;
    private final Follower follower;

    public PedroPathTelemetry(Telemetry telemetry, Follower follower) {
        this.telemetry = telemetry;
        this.follower = follower;
    }

    public void pathTelemetry(String message) {
        telemetry.addLine(message);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}

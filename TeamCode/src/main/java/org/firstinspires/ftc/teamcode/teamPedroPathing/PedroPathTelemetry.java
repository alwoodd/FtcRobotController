package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AllianceColor;

public class PedroPathTelemetry {
    private final Telemetry telemetry;
    private final Follower follower;
    private final AllianceColor currentColor;

    public PedroPathTelemetry(Telemetry telemetry, Follower follower, AllianceColor currentColor) {
        this.telemetry = telemetry;
        this.follower = follower;
        this.currentColor = currentColor;
    }

    public void pathTelemetry(String message) {
        telemetry.addData("Alliance:", currentColor.toString());
        telemetry.addLine();
        telemetry.addLine(message);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}

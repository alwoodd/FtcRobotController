package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AllianceColor;

/**
 * Standardized telemetry that shows the current alliance color, a custom message,
 * and the robot's current field position and heading.
 */
public class PedroPathTelemetry {
    private final Telemetry telemetry;
    private final Follower follower;
    private final AllianceColor currentColor;

    public PedroPathTelemetry(Telemetry telemetry, Follower follower, AllianceColor currentColor) {
        this.telemetry = telemetry;
        this.follower = follower;
        this.currentColor = currentColor;
    }

    /**
     * Shows standardized telemetry output.
     * @param message custom message, such as "Going from wall to launch zone".
     */
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

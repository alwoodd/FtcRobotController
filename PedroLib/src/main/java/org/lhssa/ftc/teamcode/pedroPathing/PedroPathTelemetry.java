package org.lhssa.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Standardized telemetry that shows the current alliance color, a custom message,
 * and the robot's current field position and heading.
 */
public class PedroPathTelemetry {
    private final Telemetry telemetry;
    private final Follower follower;
    private AllianceColor currentColor;
    private Telemetry.Item allianceColorItem;

    public PedroPathTelemetry(Telemetry telemetry, Follower follower, AllianceColor currentColor) {
        this.telemetry = telemetry;
        this.follower = follower;
        this.currentColor = currentColor;
        resetTelemetryItems();
    }

    /**
     * Shows standardized telemetry output.
     * @param message custom message, such as "Going from wall to launch zone".
     */
    public void pathTelemetry(String message) {
        if (telemetry.isAutoClear()) {
            resetTelemetryItems();
        }
        allianceColorItem.setValue(currentColor.toString());
        telemetry.addLine();
        telemetry.addLine(message);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /**
     * Set current alliance color, potentially to a different one than constructed.
     * @param allianceColor
     */
    public void setAllianceColor(AllianceColor allianceColor) {
        this.currentColor = allianceColor;
        allianceColorItem.setValue(currentColor.toString());
    }

    /**
     * (Re)Add Alliance telemetry.
     */
    private void resetTelemetryItems() {
        this.allianceColorItem = telemetry.addData("Alliance", currentColor.toString());
    }
}

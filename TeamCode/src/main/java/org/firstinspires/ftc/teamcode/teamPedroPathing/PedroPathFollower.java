package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class provides a convenient and standard way to tell the Follower which Path to follow,
 * then call follower.update() until the Path is complete.
 * The constructor also guarantees that follower.setStartingPose() is called.
 */
public class PedroPathFollower {
    private final LinearOpMode opMode;
    private final Follower follower;
    private final PedroPathTelemetry pedroTelemetry;

    /**
     * Constructor
     * @param opMode LinearOpMode instance
     * @param follower Follower instance
     * @param pedroTelemetry PedroPathTelemetry instance
     * @param startPose Pose used for call to follower.setStartingPose()
     */
    public PedroPathFollower(LinearOpMode opMode, Follower follower, PedroPathTelemetry pedroTelemetry, Pose startPose) {
        this.opMode = opMode;
        this.follower = follower;
        this.pedroTelemetry = pedroTelemetry;
        setStartingPose(startPose);
    }

    private void setStartingPose(Pose startPose) {
        follower.setStartingPose(startPose);
        follower.update();
        pedroTelemetry.pathTelemetry("Starting Pose");
    }

    /**
     * Tell follower to follow the passed pathChain,
     * then call goPath().
     * @param pathChain pathChain to follow
     * @param message message to display while path is being followed.
     */
    public void followPathChain(PathChain pathChain, String message) {
        follower.followPath(pathChain, true);
        goPath(message);
    }

    /**
     * Tell follower to follow the passed pathChain, at the passed maxPower,
     * then call goPath().
     * @param pathChain pathChain to follow
     * @param maxPower maxPower for followPath()
     * @param message message to display while path is being followed.
     */
    public void followPathChain(PathChain pathChain, double maxPower, String message) {
        follower.followPath(pathChain, maxPower, true);
        goPath(message);
    }

    /**
     * Call follower.update() until the path completes.
     * @param message message to display while path is being followed.
     */
    private void goPath(String message) {
        while (opMode.opModeIsActive() && follower.isBusy()){
            pedroTelemetry.pathTelemetry(message);
            follower.update();
        }
    }
}

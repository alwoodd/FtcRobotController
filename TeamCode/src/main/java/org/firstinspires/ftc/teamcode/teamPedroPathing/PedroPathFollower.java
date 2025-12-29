package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PedroPathFollower {
    private final LinearOpMode opMode;
    private final Follower follower;
    private final PedroPathTelemetry pedroTelemetry;

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

    public void followPathChain(PathChain pathChain, String message) {
        follower.followPath(pathChain, true);

        while (opMode.opModeIsActive() && follower.isBusy()){
            pedroTelemetry.pathTelemetry(message);
            follower.update();
        }
    }

/*
    private void pathTelemetry() {
        Telemetry telemetry = opMode.telemetry;
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private void pathTelemetry(String message) {
        opMode.telemetry.addLine(message);
        pathTelemetry();
    }
*/
}

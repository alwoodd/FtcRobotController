package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroSleep;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroTeleopData;

/**
 * This OpMode demonstrates using Pedro Pathing for teleOp,
 * and also building and a Path to "instantly" go to the launch Pose
 * from anywhere on the field.
 */
@TeleOp(name = "Pedro Path Teleop")
public class PedroPathTeleOp extends LinearOpMode {
    enum FollowPathDestination {
        LAUNCH,
        PARK,
        NONE
    }
    private FollowPathDestination followPathDestination;
    private Follower follower;

    private Pose launchPose;// = new Pose(90, 90, Math.toRadians(45));
    private Pose parkPose;

    PedroPathTelemetry pedroPathTelemetry;
    PedroSleep pedroSleep;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);

        follower = pedroPathConfiguration.getFollower();
        follower.setStartingPose(PedroTeleopData.startingPose == null ? new Pose() :
                PedroTeleopData.startingPose);
        AllianceColor allianceColor = PedroTeleopData.allianceColor == null ? AllianceColor.RED :
                PedroTeleopData.allianceColor;
        setPoses(allianceColor);
        pedroPathTelemetry = new PedroPathTelemetry(telemetry, follower, allianceColor);
        pedroSleep = new PedroSleep(follower);

        follower.startTeleOpDrive(); //This calls update() as well.
        pedroPathTelemetry.pathTelemetry("");

        waitForStart();

        /*
         * The follower can be either in startTeleOpDrive() or followPath().
         */
        String pedroMessage = "";
        while(opModeIsActive()) {
            follower.update();
            pedroPathTelemetry.pathTelemetry(pedroMessage);

            //Call setTeleOpDrive() as long as isTeleopDrive().
            if (follower.isTeleopDrive()) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                pedroMessage = "TeleOp Mode";
            }
            /*
             * If not isTeleopDrive(), then we might still be following a Path.
             * If that Path is complete (not isBusy()), performPathEndActions(),
             * then re-startTeleOpDrive().
             */
            else if (!follower.isBusy()) {
                performPathEndActions();
                follower.startTeleOpDrive();
            }

            if (gamepad1.xWasPressed()) {
                pedroMessage = toggleFollowPath(fromHereToLaunch(), FollowPathDestination.LAUNCH);
            }

            if (gamepad1.yWasPressed()) {
               //startBallPickup();
            }

            if (gamepad1.aWasPressed()) {
                pedroMessage = toggleFollowPath(fromHereToPark(), FollowPathDestination.PARK);
            }
        }
    }

    /**
     * Perform whatever actions are required for the current followPathDestination.
     */
    private void performPathEndActions() {
        switch (followPathDestination) {
            case LAUNCH:
                shootBalls();
                break;
            case PARK:
                break;
        }

        followPathDestination = FollowPathDestination.NONE;
    }

    private void shootBalls() {
        pedroPathTelemetry.pathTelemetry("Shooting Balls");
        pedroSleep.sleep(4000);
    }

    /**
     * If follower not following a path, follow the passed path.
     * Otherwise, if the follower *is* following a path, startTeleOpDrive().
     * @param path PathChain to follow
     * @param followPathDestination requiring this param ensures it gets set!
     * @return string to set pedroMessage to.
     */
    private String toggleFollowPath(PathChain path, FollowPathDestination followPathDestination) {
        String pedroMessage;

        if (!follower.isBusy()) {
            this.followPathDestination = followPathDestination;
            follower.followPath(path);
            pedroMessage = "Follow Path Mode";
        }
        else {
            follower.startTeleOpDrive();
            pedroMessage = "TeleOp Mode";
            this.followPathDestination = FollowPathDestination.NONE;
        }

        return pedroMessage;
    }

    /**
     * Build a PathChain from our current Pose to the launchPose,
     * and set it to LinearHeadingInterpolation.
     * @return PathChain
     */
    private PathChain fromHereToLaunch() {
        Pose herePose = follower.getPose();

        return follower.pathBuilder()
            .addPath(new BezierLine(herePose, launchPose))
            .setLinearHeadingInterpolation(herePose.getHeading(), launchPose.getHeading())
            .build();
    }

    /**
     * Build a PathChain from our current Pose to the parkPose,
     * and set it to LinearHeadingInterpolation.
     * @return PathChain
     */
    private PathChain fromHereToPark() {
        Pose herePose = follower.getPose();

        return follower.pathBuilder()
            .addPath(new BezierLine(herePose, parkPose))
            .setLinearHeadingInterpolation(herePose.getHeading(), parkPose.getHeading())
            .build();
    }

    /**
     * Set required fixed Poses, using the red or blue AutonomousPaths.
     * @param allianceColor
     */
    private void setPoses(AllianceColor allianceColor) {
        AutonomousPaths paths = (allianceColor == AllianceColor.RED) ?
            new RedPedroPaths(follower) : new BluePedroPaths(follower, new RedPedroPaths(follower));

        launchPose = paths.pathFromBackWallToLaunchZone().endPose();
        parkPose = paths.parkPose();
    }
}

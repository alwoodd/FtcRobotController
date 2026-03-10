package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPather;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroSleep;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroTeleopData;

import java.util.List;

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
    static class ShooterSpeed {
        final double speed;
        final String speedDescription;
        ShooterSpeed(double speed, String speedDescription) {
            this.speed = speed;
            this. speedDescription = speedDescription;
        }

        @Override
        public String toString() {
            return "Current shooter speed is " + this.speedDescription;
        }
    }
    static class ShooterSpeeds {
        private final List<ShooterSpeed> shooterSpeeds;
        private int index = 0;

        ShooterSpeeds(ShooterSpeed... speeds) {
            this.shooterSpeeds = List.of(speeds);
        }

        ShooterSpeed getFirst() {
            index = 0;
            return shooterSpeeds.get(index);
        }

        ShooterSpeed next() {
            index = (index + 1) % shooterSpeeds.size();
            return shooterSpeeds.get(index);
        }
    }

    private FollowPathDestination followPathDestination;
    private Follower follower;
    private AllianceColor allianceColor;
    private PedroPather teamPaths;
    private PedroPathTelemetry pedroPathTelemetry;
    private PedroSleep pedroSleep;

    @Override
    public void runOpMode() throws InterruptedException {
        //RobotHardware robot = new RobotHardware(this);

        ShooterSpeeds shooterSpeeds = new ShooterSpeeds(
                new ShooterSpeed(.2, "Low"),
                new ShooterSpeed(.5, "Middlin'"),
                new ShooterSpeed(1, "Full Blast")
        );

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);

        follower = pedroPathConfiguration.getFollower();
        follower.setStartingPose(PedroTeleopData.startingPose == null ? new Pose() : PedroTeleopData.startingPose);
        pedroSleep = new PedroSleep(follower, 10);
        allianceColor = PedroTeleopData.allianceColor == null ? AllianceColor.RED :
                PedroTeleopData.allianceColor;
        pedroPathTelemetry = new PedroPathTelemetry(telemetry, follower, allianceColor);
        initSetup();
        teamPaths = new PedroPather(TeamPoses.canonicalColor, allianceColor);
        if (PedroTeleopData.startingPose == null) {
            follower.setStartingPose(TeamPoses.frontWallStartingPose);
        }

        ShooterSpeed currentShooterSpeed = shooterSpeeds.getFirst();
        String pedroMessage = "Current shooter speed is " + currentShooterSpeed.speedDescription;

        follower.startTeleOpDrive(); //This calls update() as well.
        waitForStart();

        //The follower can be either in startTeleOpDrive() or followPath().
        while (opModeIsActive()) {
            follower.update();
            pedroPathTelemetry.pathTelemetry(pedroMessage);

            //Call setTeleOpDrive() as long as isTeleopDrive().
            if (follower.isTeleopDrive()) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                //pedroMessage = "TeleOp Mode";
            }
             /* If not isTeleopDrive(), then we might still be following a Path.
             * If that Path is complete (not isBusy()), performPathEndActions(),
             * then re-startTeleOpDrive().
             */
            else if (!follower.isBusy()) {
                fineTuneHeading();
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

            if (gamepad1.b) {
                //robot.shoot(currentShooterSpeed.speed);
            }
            else {
                //robot.shoot(0);
            }

            if (gamepad1.rightBumperWasPressed()) {
                currentShooterSpeed = shooterSpeeds.next();
                pedroMessage = currentShooterSpeed.toString();
            }
        }
    }

    /**
     * Give the follower a few more updates.
     */
    private void fineTuneHeading() {
        pedroPathTelemetry.pathTelemetry("Fine Tune Heading");
        follower.followPath(follower.getCurrentPath());
        pedroSleep.sleep(500);
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
        pedroSleep.sleep(2000);
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
            .addPath(new BezierLine(herePose, TeamPoses.backGoalShootPose))
            .setLinearHeadingInterpolation(herePose.getHeading(), TeamPoses.backGoalShootPose.getHeading())
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
            .addPath(new BezierLine(herePose, TeamPoses.parkPose))
            .setLinearHeadingInterpolation(herePose.getHeading(), TeamPoses.parkPose.getHeading())
            .build();
    }

    /**
     * Initialization time setup.
     */
    private void initSetup() {
        telemetry.setAutoClear(false);
        pedroPathTelemetry.pathTelemetry("Initialize Setup");

        telemetry.addLine("Press Right Bumper to toggle between Red and Blue alliance.");
        Telemetry.Item allianceColorItem = telemetry.addData(allianceColor.toString(), " currently selected");
        if (PedroTeleopData.startingPose == null) {
            telemetry.addLine();
            telemetry.addLine("Starting Pose not set during Autonomous");
            telemetry.addLine("Defaulting to audience-side start Pose");
        }

        while (opModeInInit()) {
            allianceColorItem.setCaption(allianceColor.toString());
            pedroPathTelemetry.setAllianceColor(allianceColor);
            telemetry.update();

            allianceColor = AllianceColor.INSTANCE.toggleColor(gamepad1.rightBumperWasPressed(), allianceColor);
        }
        telemetry.setAutoClear(true);
    }
}

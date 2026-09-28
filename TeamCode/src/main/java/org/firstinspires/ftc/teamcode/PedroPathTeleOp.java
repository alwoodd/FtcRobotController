package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConfiguration;
import org.firstinspires.ftc.teamcode.teamPedroPathing.TeamPoses;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPathTelemetry;
import org.lhssa.ftc.teamcode.pedroPathing.PedroSleep;
import org.lhssa.ftc.teamcode.pedroPathing.PedroTeleopData;
import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;

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

        @NonNull
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
    private PedroPathTelemetry pedroPathTelemetry;
    private PedroSleep pedroSleep;
    private String pedroMessage;

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
        follower.setPose(PedroTeleopData.startingPose == null ? TeamPoses.startPose : PedroTeleopData.startingPose);
        pedroSleep = new PedroSleep(follower, 10);
        allianceColor = PedroTeleopData.allianceColor == null ? AllianceColor.RED :
                PedroTeleopData.allianceColor;
        pedroPathTelemetry = new PedroPathTelemetry(telemetry, follower, allianceColor);
        initSetup();
        //PedroPather teamPaths = new PedroPather(TeamPoses.canonicalColor, allianceColor);

        ShooterSpeed currentShooterSpeed = shooterSpeeds.getFirst();
        pedroMessage = "Current shooter speed is " + currentShooterSpeed.speedDescription;

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

    //RobotLog.ii("PedroPathTeleOpLog", "following? %b; holding? %b; manual? %b; idle? %b; isBusy? %b; atParametricEnd? %b",
    //            follower.following(), follower.holding(), follower.manual(), follower.idle(), follower.isBusy(), follower.atParametricEnd());

            pedroPathTelemetry.pathTelemetry(pedroMessage);

            //If we're not following a path...
            if (!follower.isBusy()) {
                //And we're currently holding at the end of a path...
                if (/*follower.atParametricEnd() &&*/ follower.holding()) {
                    performPathEndActions();
                }
                follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                pedroMessage = "TeleOp Mode";
            }

            if (gamepad1.xWasPressed()) {
                toggleFollowPath(fromHereToLaunch(), FollowPathDestination.LAUNCH);
            }

            if (gamepad1.yWasPressed()) {
               //startBallPickup();
            }

            if (gamepad1.aWasPressed()) {
                toggleFollowPath(fromHereToPark(), FollowPathDestination.PARK);
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
/*
    private void fineTuneHeading() {
        pedroPathTelemetry.pathTelemetry("Fine Tune Heading");
        follower.followPath(follower.getCurrentPath());
        pedroSleep.sleep(500);
    }
*/

    /**
     * Perform whatever actions are required for the current followPathDestination.
     */
    private void performPathEndActions() {
    //RobotLog.ii("PedroPathTeleOpLog", "performPathEndActions with followPathDestination of %s",
    //        followPathDestination.toString());
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
    private void toggleFollowPath(Path path, FollowPathDestination followPathDestination) {
//RobotLog.ii("PedroPathTeleOpLog", "toggleFollowPath()");
        if (!follower.isBusy()) {
            this.followPathDestination = followPathDestination;
            follower.follow(path);
            pedroMessage = "Follow Path Mode";
        }
        else {
            this.followPathDestination = FollowPathDestination.NONE;
            follower.hold(follower.pose());
        }
    }

    /**
     * Build a PathChain from our current Pose to the launchPose,
     * and set it to LinearHeadingInterpolation.
     * @return PathChain
     */
    private Path fromHereToLaunch() {
        Pose herePose = follower.pose();

        return Paths.line(herePose, TeamPoses.backGoalShootPose).linear(herePose, TeamPoses.backGoalShootPose);
    }

    /**
     * Build a PathChain from our current Pose to the parkPose,
     * and set it to LinearHeadingInterpolation.
     * @return PathChain
     */
    private Path fromHereToPark() {
        Pose herePose = follower.pose();

        return Paths.line(herePose, TeamPoses.parkPose).linear(herePose, TeamPoses.parkPose);
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

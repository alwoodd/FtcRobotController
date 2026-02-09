package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode demonstrates using Pedro Pathing for teleOp,
 * and also building and a Path to "instantly" go to the launch Pose
 * from anywhere on the field.
 */
@TeleOp(name = "Pedro Path Teleop")
public class PedroPathTeleOp extends LinearOpMode {
    /*
     * The preceding Autonomous OpMode should set this static startPose
     * with the Pose it stopped at (follower.getPose()).
     */
    public static Pose startPose;
    private Follower follower;

    private final Pose launchPose = new Pose(90, 90, Math.toRadians(45));

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);

        follower = pedroPathConfiguration.getFollower();
        follower.setStartingPose(startPose == null? new Pose() : startPose);
        follower.startTeleOpDrive(); //This calls update() as well.

        waitForStart();

        /*
         * The follower can be either in startTeleOpDrive() or followPath().
         */
        while(opModeIsActive()) {
            follower.update();

            //Call setTeleOpDrive() as long as isTeleopDrive().
            if (follower.isTeleopDrive()) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            }
            /*
             * If not isTeleopDrive(), then we might still be following a Path.
             * If that Path is complete (not isBusy()), do whatever we went there to do,
             * then re-startTeleOpDrive().
             */
            else if (!follower.isBusy()) {
                //shootBalls();
                follower.startTeleOpDrive();
            }

            /*
             * See if we pressed the gamepad button to follow a Path.
             * If whatever Path we might already have been on is complete (not isBusy()),
             * followPath() the path fromHereToLaunch().
             */
            if (gamepad1.xWasPressed() && !follower.isBusy()) {
                follower.followPath(fromHereToLaunch());
            }

            if (gamepad1.yWasPressed()) {
               //startBallPickup();
            }
        }
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
}

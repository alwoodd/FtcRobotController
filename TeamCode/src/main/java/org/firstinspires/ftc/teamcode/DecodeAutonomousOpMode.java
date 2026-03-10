package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroMotion;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroSleep;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroTeleopData;
import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;
import org.lhssa.ftc.teamcode.pedroPathing.HeadingInterpolationType;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPather;

/**
 * This OpMode demonstrates:
 * -Running the OpMode in a loop (while(opModeIsActive()).
 * -Using a state machine to control actions to be performed during each iteration.
 * -A way to flexibly set a start Pose on a configured teleOp class, to its configured
 *  static field the teleOp class uses for its start Pose.
 */
@Autonomous(name = "Autonomous with actions Ex")
public class DecodeAutonomousOpMode extends LinearOpMode {
    private PedroPathTelemetry pedroTelemetry;
    private Follower follower;
    private final double ballPickupPower = .4;
    private AllianceColor selectedColor = AllianceColor.RED;
    private BallSpikeLocation ballSpikeLocation = BallSpikeLocation.GOAL_SIDE;
    private FrontBackLocation startLocation = FrontBackLocation.FRONT;
    private FrontBackLocation launchLocation = FrontBackLocation.BACK;
    private int pathState = 0;
    private RobotHardware robot;
    private PedroPather teamPaths;
    private PedroSleep pedroSleep;
    private PedroMotion motion;

    private Pose startPose;
    private Pose launchPose;
    private Pose startBallPickupPose;
    private Pose endBallPickupPose;
    private Pose leavePose;

    private String pedroMessage = "NO MESSAGE";

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(this);
        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);
        follower = pedroPathConfiguration.getFollower();
        pedroSleep = new PedroSleep(follower);
        motion = new PedroMotion(follower);

        initSetup();

        teamPaths = new PedroPather(TeamPoses.canonicalColor, selectedColor);
        setPoses();
        pedroTelemetry = new PedroPathTelemetry(telemetry, follower, selectedColor);
        PedroTeleopData.allianceColor = selectedColor;

        follower.setStartingPose(teamPaths.normalizePose(startPose));

        waitForStart();

        boolean actionsDone = false;
        while (opModeIsActive() && !actionsDone) {
            follower.update();
            actionsDone = performActions();
            pedroTelemetry.pathTelemetry(pedroMessage);
        }

        PedroTeleopData.startingPose = follower.getPose();
    }

    private boolean performActions() {
        boolean actionsDone = false;
        switch (pathState) {
            case 0:
                this.pedroMessage = "Going from wall to launch zone";
                motion.goPath(teamPaths.pathBetween(startPose, launchPose));
                if (motion.isPathComplete()) {
                    shootBalls();
                    incrementPathState(motion.isPathComplete());
                }
                break;
            case 1:
                this.pedroMessage = "Going from launch zone to ball pickup";
                motion.goPath(teamPaths.pathBetween(launchPose, startBallPickupPose));
                incrementPathState(motion.isPathComplete());
                break;
            case 2:
                this.pedroMessage = "Picking up balls";
                motion.goPath(teamPaths.pathBetween(startBallPickupPose, endBallPickupPose, HeadingInterpolationType.CONSTANT), ballPickupPower);
                if (motion.isPathComplete()) {
                    ballPickup();
                    incrementPathState(motion.isPathComplete());
                }
                break;
            case 3:
                this.pedroMessage = "Going from ball pickup to launch zone";
                motion.goPath(teamPaths.pathBetween(endBallPickupPose, launchPose));
                if (motion.isPathComplete()) {
                    shootBalls();
                    incrementPathState(motion.isPathComplete());
                }
                break;
            case 4:
                this.pedroMessage = "Leaving";
                motion.goPath(teamPaths.pathBetween(launchPose, leavePose));
                incrementPathState(motion.isPathComplete());
                break;
            case 5:
                if (!follower.isBusy()) {
                    actionsDone = true;
                }
                break;
        }
        return actionsDone;
    }

    //Emulate shooting balls.
    private void shootBalls(){
        pedroTelemetry.pathTelemetry("Shooting balls.");
        pedroSleep.sleep(3000);
    }

    //Emulate pickup up balls.
    private void ballPickup(){
        pedroTelemetry.pathTelemetry("Ball scooper turned on.");
        pedroSleep.sleep(2000);
    }

    private void incrementPathState(boolean isPathComplete) {
        if (isPathComplete) pathState++;
    }

    private void setPoses() {
        startPose = (startLocation == FrontBackLocation.BACK) ? TeamPoses.backWallStartingPose :
            TeamPoses.frontWallStartingPose;

        launchPose = (launchLocation == FrontBackLocation.BACK) ? TeamPoses.backGoalShootPose :
            TeamPoses.frontGoalShootPose;

        switch (ballSpikeLocation) {
            case GOAL_SIDE:
                startBallPickupPose = TeamPoses.startGoalBallPickupPose;
                endBallPickupPose = TeamPoses.endGoalBallPickupPose;
                    leavePose = TeamPoses.startMiddleBallPickupPose;
                break;
            case MIDDLE:
                startBallPickupPose = TeamPoses.startMiddleBallPickupPose;
                endBallPickupPose = TeamPoses.endMiddleBallPickupPose;
                leavePose = TeamPoses.startAudienceBallPickupPose;
                break;
            case AUDIENCE_SIDE:
                startBallPickupPose = TeamPoses.startAudienceBallPickupPose;
                endBallPickupPose = TeamPoses.endAudienceBallPickupPose;
                leavePose = TeamPoses.startMiddleBallPickupPose;
                break;
        }
    }

    /**
     * Call AllianceColor.INSTANCE.toggleColor(), BallSpikeLocation.INSTANCE.toggleLocation(),
     * and toggleMaxPower() as long as opModeInInit().
     */
    private void initSetup() {
        while (opModeInInit()) {
            telemetry.addLine("Press Right Bumper to toggle between Red and Blue alliance.");
            //telemetry.addLine("Press Right Bumper to confirm selection.");
            telemetry.addData(selectedColor.toString(), " currently selected");
            telemetry.addLine();
            telemetry.addLine("Press Left Bumper to toggle set ball pickup location");
            telemetry.addData(ballSpikeLocation.toString(), " currently selected");
            telemetry.addLine();
            telemetry.addLine("Press Y to toggle START between Front and Back");
            telemetry.addData(startLocation.toString(), "currently selected");
            telemetry.addLine();
            telemetry.addLine("Press X to toggle LAUNCH between Front and Back");
            telemetry.addData(launchLocation.toString(), "currently selected");
            telemetry.update();

            selectedColor = AllianceColor.INSTANCE.toggleColor(gamepad1.rightBumperWasPressed(), selectedColor);
            ballSpikeLocation = BallSpikeLocation.INSTANCE.toggleLocation(gamepad1.leftBumperWasPressed(), ballSpikeLocation);
            startLocation = FrontBackLocation.INSTANCE.toggleLocation(gamepad1.yWasPressed(), startLocation);
            launchLocation = FrontBackLocation.INSTANCE.toggleLocation(gamepad1.xWasPressed(), launchLocation);
        }
    }
}
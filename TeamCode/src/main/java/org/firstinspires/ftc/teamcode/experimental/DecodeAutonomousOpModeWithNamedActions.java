package org.firstinspires.ftc.teamcode.experimental;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.AutonomousPaths;
import org.firstinspires.ftc.teamcode.BallSpikeLocation;
import org.firstinspires.ftc.teamcode.BluePedroPaths;
import org.firstinspires.ftc.teamcode.FrontBackLocation;
import org.firstinspires.ftc.teamcode.PedroPathConfiguration;
import org.firstinspires.ftc.teamcode.RedPedroPaths;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroSleep;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroTeleopData;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * This OpMode demonstrates:
 * -Running the OpMode in a loop (while(opModeIsActive()).
 * -Using a state machine to control actions to be performed during each iteration.
 * -A way to flexibly set a start Pose on a configured teleOp class, to its configured
 *  static field the teleOp class uses for its start Pose.
 */
@Autonomous(name = "Autonomous with named actions")
@Disabled
public class DecodeAutonomousOpModeWithNamedActions extends LinearOpMode {
    enum ActionStep {
        WALL_TO_LAUNCH,
        LAUNCH_TO_BALL_PICKUP,
        START_TO_END_BALL_PICKUP,
        BALL_PICKUP_TO_LAUNCH,
        LAUNCH_TO_LEAVE,
        SHOOT_TO_LEAVE,
        DONE
    }

    private PedroPathTelemetry pedroTelemetry;
    private Follower follower;
    private final double ballPickupPower = .3;
    private double currentMaxPower, defaultMaxPower;
    private final double maxMaxPower = 1;
    private AllianceColor selectedColor = AllianceColor.RED;
    private BallSpikeLocation ballSpikeLocation = BallSpikeLocation.GOAL_SIDE;
    private FrontBackLocation startLocation = FrontBackLocation.FRONT;
    //private FrontBackLocation launchLocation = FrontBackLocation.BACK;

    private List<ActionStep> actionSteps;
    private Iterator<ActionStep> actionStepIterator;
    private ActionStep currentStep;
    //private int pathState = 0;

    private RobotHardware robot;
    private AutonomousPaths pedroPaths;
    private PedroSleep pedroSleep;

    private PathChain pathFromStartToLaunchZone;
    private PathChain pathFromLaunchZoneToStartBallPickup;
    private PathChain pathFromStartBallPickupToEndBallPickup;
    private PathChain pathFromEndBallPickupToLaunchZone;
    private PathChain pathFromLaunchZoneToLeave;

    private String pedroMessage = "NO MESSAGE";

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(this);
        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);
        follower = pedroPathConfiguration.getFollower();
        pedroSleep = new PedroSleep(follower);

        defaultMaxPower = follower.getMaxPowerScaling();
        currentMaxPower = defaultMaxPower;
        initSetup();
        follower.setMaxPower(currentMaxPower);

        pedroPaths = createPedroPaths(selectedColor);
        setBallSpikeLocationPaths();
        setRobotStartPath();
        pedroTelemetry = new PedroPathTelemetry(telemetry, follower, selectedColor);
        PedroTeleopData.allianceColor = selectedColor;

        follower.setStartingPose(pedroPaths.frontWallstartingPose());

        actionStepIterator = actionSteps.iterator();
        currentStep = actionStepIterator.next();
        waitForStart();

        boolean done;
        while (opModeIsActive()) {
            follower.update();
            done = performActions();
            pedroTelemetry.pathTelemetry(pedroMessage);
            if (done) break;
/*
            telemetry.addLine();
            RobotHardware.SpinnerVelocities spinnerVelocities = robot.getSpinnerVelocities();
            telemetry.addData("Left Spinner Velocity", spinnerVelocities.left);
            telemetry.addData("Right Spinner Velocity", spinnerVelocities.right);
            telemetry.setAutoClear(false); //Add above telemetry to pedroTelemetry.
            telemetry.update();
            telemetry.setAutoClear(true);
*/
        }

        PedroTeleopData.startingPose = follower.getPose();
    }

    private boolean performActions() {
        boolean done = false;
        switch (currentStep) {
            case WALL_TO_LAUNCH:
                if (!follower.isBusy()) {
                    pedroMessage = "Going from wall to launch zone";
                    follower.followPath(pathFromStartToLaunchZone);
                    //pathState++;
                    currentStep = actionStepIterator.next();
                }
                break;
            case LAUNCH_TO_BALL_PICKUP:
                if (!follower.isBusy()) {
                    shootBalls();
                    pedroMessage = "Going from launch zone to ball pickup";
                    follower.followPath(pathFromLaunchZoneToStartBallPickup);
                    //pathState++;
                }
                break;
            case START_TO_END_BALL_PICKUP:
                if (!follower.isBusy()) {
                    ballPickup();
                    pedroMessage = "Picking up balls";
                    follower.followPath(pathFromStartBallPickupToEndBallPickup, ballPickupPower, true);
                    //pathState++;
                }
                break;
            case BALL_PICKUP_TO_LAUNCH:
                if (!follower.isBusy()) {
                    pedroMessage = "Going from ball pickup to launch zone";
                    follower.followPath(pathFromEndBallPickupToLaunchZone);
                    //pathState++;
                }
                break;
            case LAUNCH_TO_LEAVE:
                if (!follower.isBusy()) {
                    shootBalls();
                    pedroMessage = "Leaving";
                    follower.followPath(pathFromLaunchZoneToLeave);
                    //pathState++;
                }
                break;
            case SHOOT_TO_LEAVE:
                if (!follower.isBusy()) {
                    pedroMessage = "Leaving";
                    //???follower.followPath();
                }
                break;
            case DONE:
                if (!follower.isBusy()) {
                    done = true;
                }
                break;
        }
        return done;
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

    /**
     * Set the spike location-dependent PathChains.
     */
    private void setBallSpikeLocationPaths() {
        switch (ballSpikeLocation) {
            case AUDIENCE_SIDE:
                pathFromLaunchZoneToStartBallPickup = pedroPaths.pathFromLaunchZoneToAudienceSideBallPickup();
                pathFromStartBallPickupToEndBallPickup = pedroPaths.pathFromAudienceSideBallPickupToEndBallPickup();
                pathFromEndBallPickupToLaunchZone = pedroPaths.pathFromAudienceSideEndBallPickupToLaunchZone();
                pathFromLaunchZoneToLeave = pedroPaths.pathFromLaunchZoneToMiddleSideLeave();
                break;
            case MIDDLE:
                pathFromLaunchZoneToStartBallPickup = pedroPaths.pathFromLaunchZoneToMiddleSideBallPickup();
                pathFromStartBallPickupToEndBallPickup = pedroPaths.pathFromMiddleSideBallPickupToEndBallPickup();
                pathFromEndBallPickupToLaunchZone = pedroPaths.pathFromMiddleSideEndBallPickupToLaunchZone();
                pathFromLaunchZoneToLeave = pedroPaths.pathFromLaunchZoneToAudienceSideLeave();
                break;
            case GOAL_SIDE:
                pathFromLaunchZoneToStartBallPickup = pedroPaths.pathFromLaunchZoneToGoalSideBallPickup();
                pathFromStartBallPickupToEndBallPickup = pedroPaths.pathFromGoalSideBallPickupToEndBallPickup();
                pathFromEndBallPickupToLaunchZone = pedroPaths.pathFromGoalSideEndBallPickupToLaunchZone();
                pathFromLaunchZoneToLeave = pedroPaths.pathFromLaunchZoneToGoalSideLeave();
                break;
            case NONE:
                pathFromLaunchZoneToLeave = pedroPaths.pathFromFrontLaunchZoneToLeave();
                break;
        }
    }

    private void setRobotStartPath() {
        if (ballSpikeLocation != BallSpikeLocation.NONE) {
            if (startLocation == FrontBackLocation.FRONT) {
                pathFromStartToLaunchZone = pedroPaths.pathFromFrontWallToLaunchZone();
            }
            else {
                pathFromStartToLaunchZone = pedroPaths.pathFromBackWallToLaunchZone();
            }
        }
        else {
            pathFromStartToLaunchZone = pedroPaths.pathFromFrontWallToFrontLaunchZone();
        }
    }

/*
    private void setRobotLaunchLocation() {
        if (launchLocation == FrontBackLocation.FRONT) {
            pathFromStartToLaunchZone = pedroPaths.pathFromFrontWallToLaunchZone();
        }
        else {
            pathFromStartToLaunchZone = pedroPaths.pathFromBackWallToLaunchZone();
        }
    }
*/

    /**
     * Depending on the passed selectedColor, instantiate and return either a
     * RedPedroPathsFrontWall or BluePedroPathsFrontWall.
     * Both implement the AutonomousPedroPathsFrontWall interface.
     * @param selectedColor RED or BLUE
     * @return AutonomousPedroPathsFrontWall
     */
    private AutonomousPaths createPedroPaths(AllianceColor selectedColor) {
        if (selectedColor == AllianceColor.RED) {
            return new RedPedroPaths(follower);
        }
        else {
            return new BluePedroPaths(follower, new RedPedroPaths(follower));
        }
    }

    /**
     * Call AllianceColor.INSTANCE.toggleColor(), BallSpikeLocation.INSTANCE.toggleLocation(),
     * and toggleMaxPower() as long as opModeInInit().
     */
    private void initSetup() {
        while (opModeInInit()) {
            telemetry.addLine("Press Right Bumper to toggle between Red and Blue alliance.");
            telemetry.addData(selectedColor.toString(), " currently selected");
            telemetry.addLine();
            telemetry.addLine("Press Y to toggle START between Front and Back");
            telemetry.addData(startLocation.toString(), "currently selected");
/*
            telemetry.addLine();
            telemetry.addLine("Press A to toggle LAUNCH between Front and Back");
            telemetry.addData(launchLocation.toString(), "currently selected");
*/
            telemetry.addLine();
            telemetry.addLine("Press Left Bumper to toggle set ball pickup location");
            telemetry.addData(ballSpikeLocation.toString(), " currently selected");

            telemetry.addLine();
            telemetry.addLine("Press X to toggle max speed between " + defaultMaxPower + " and " + maxMaxPower);
            telemetry.addData(String.valueOf(currentMaxPower), "currently selected");

            telemetry.update();

            selectedColor = AllianceColor.INSTANCE.toggleColor(gamepad1.rightBumperWasPressed(), selectedColor);
            ballSpikeLocation = BallSpikeLocation.INSTANCE.toggleLocation(gamepad1.leftBumperWasPressed(), ballSpikeLocation);
            startLocation = FrontBackLocation.INSTANCE.toggleLocation(gamepad1.yWasPressed(), startLocation);
//            launchLocation = FrontBackLocation.INSTANCE.toggleLocation(gamepad1.aWasPressed(), launchLocation);
            toggleMaxPower(gamepad1.xWasPressed());
        }

        actionSteps = new ArrayList<>();
        actionSteps.add(ActionStep.WALL_TO_LAUNCH);
        if (ballSpikeLocation != BallSpikeLocation.NONE) {
            actionSteps.add(ActionStep.LAUNCH_TO_BALL_PICKUP);
            actionSteps.add(ActionStep.START_TO_END_BALL_PICKUP);
            actionSteps.add(ActionStep.BALL_PICKUP_TO_LAUNCH);
            actionSteps.add(ActionStep.LAUNCH_TO_LEAVE);
        }
        else {
            actionSteps.add(ActionStep.SHOOT_TO_LEAVE);
        }
        actionSteps.add(ActionStep.DONE);
    }

    /**
     * Toggle currentMaxPower between defaultMaxPower and maxMaxPower.
     * @param buttonPressed button result passed by the caller
     */
    private void toggleMaxPower(boolean buttonPressed) {
        if (buttonPressed) {
            if (currentMaxPower == defaultMaxPower) {
                currentMaxPower = maxMaxPower;
            }
            else {
                currentMaxPower = defaultMaxPower;
            }
        }
    }
}

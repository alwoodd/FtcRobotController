package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathFollower;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;

@Autonomous(name = "Autonomous")
public class DecodeAutonomousOpMode extends LinearOpMode {
    private double currentMaxPower, defaultMaxPower;
    private final double maxMaxPower = 1;

    private PedroPathTelemetry pedroTelemetry;
    private Follower follower;
    private final double ballPickupPower = .2;
    private AllianceColor selectedColor = AllianceColor.RED;
    private BallSpikeLocation ballSpikeLocation = BallSpikeLocation.AUDIENCE_SIDE;
    private StartLocation startLocation = StartLocation.FRONT;
    private AutonomousPaths pedroPaths;

    private Pose startingPose;
    private PathChain pathFromStartToLaunchZone;
    private PathChain pathFromLaunchZoneToStartBallPickup;
    private PathChain pathFromStartBallPickupToEndBallPickup;
    private PathChain pathFromEndBallPickupToLaunchZone;
    private PathChain pathFromLaunchZoneToLeave;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);

        follower = pedroPathConfiguration.getFollower();

        /*
         * This block, along with toggleMaxPower(), shows one way to toggle max power for the run
         * between 2 powers.
         */
        defaultMaxPower = follower.getMaxPowerScaling();
        currentMaxPower = defaultMaxPower;
        initSetup();
        follower.setMaxPower(currentMaxPower);

        pedroPaths = createPedroPaths(selectedColor);
        setBallSpikeLocationPaths();
        setStartLocation();
        pedroTelemetry = new PedroPathTelemetry(telemetry, follower, selectedColor);
        PedroPathFollower pedroPathFollower = new PedroPathFollower(this, follower, pedroTelemetry, startingPose);

        waitForStart();

        if (opModeIsActive()) {
            pedroPathFollower.followPathChain(pathFromStartToLaunchZone, "Going from wall to launch zone");
            shootBalls();
            pedroPathFollower.followPathChain(pathFromLaunchZoneToStartBallPickup, "Going from launch zone to ball pickup");
            ballPickup();
            pedroPathFollower.followPathChain(pathFromStartBallPickupToEndBallPickup, ballPickupPower, "Picking up balls");
            pedroPathFollower.followPathChain(pathFromEndBallPickupToLaunchZone, "Going from ball pickup to launch zone");
            shootBalls();
            pedroPathFollower.followPathChain(pathFromLaunchZoneToLeave, "Leaving");
        }
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
        }
    }

    private void setStartLocation() {
        if (startLocation == StartLocation.FRONT) {
            pathFromStartToLaunchZone = pedroPaths.pathFromFrontWallToLaunchZone();
            startingPose = pedroPaths.frontWallstartingPose();
        }
        else {
            pathFromStartToLaunchZone = pedroPaths.pathFromBackWallToLaunchZone();
            startingPose = pedroPaths.backWallstartingPose();
        }
    }

    /**
     * Depending on the passed selectedColor, instantiate and return either a
     * RedPedroPathsFrontWall or BluePedroPathsFrontWall.
     * Both implement the AutonomousPathsFrontWall interface.
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

    //Emulate shooting balls.
    private void shootBalls(){
        pedroTelemetry.pathTelemetry("Shooting balls.");
        sleep(4000);
    }

    //Emulate pickup up balls.
    private void ballPickup(){
        pedroTelemetry.pathTelemetry("Ball scooper turned on.");
        sleep(2000);
    }

    /**
     * Perform pre-start initializations as long as opModeInInit().
     * Get the AllianceColor, and target BallSpikeLocation.
     * Also toggleMaxPower().
     */
    private void initSetup() {
        while (opModeInInit()) {
            telemetry.addLine("Press Right Bumper to toggle between Red and Blue alliance.");
            telemetry.addData(selectedColor.toString(), " currently selected");
            telemetry.addLine();
            telemetry.addLine("Press Left Bumper to toggle set ball pickup location");
            telemetry.addData(ballSpikeLocation.toString(), " currently selected");
            telemetry.addLine();
            telemetry.addLine("Press Y to toggle between Front and Back starting location");
            telemetry.addData(ballSpikeLocation.toString(), " currently selected");
            telemetry.addLine();
            telemetry.addLine("Press X to toggle max speed between " + defaultMaxPower + " and " + maxMaxPower);
            telemetry.addData(String.valueOf(currentMaxPower), "currently selected");
            telemetry.update();

            selectedColor = AllianceColor.INSTANCE.toggleColor(gamepad1.rightBumperWasPressed(), selectedColor);
            ballSpikeLocation = BallSpikeLocation.INSTANCE.toggleLocation(gamepad1.leftBumperWasPressed(), ballSpikeLocation);
            startLocation = StartLocation.INSTANCE.toggleLocation(gamepad1.yWasPressed(), startLocation);
            toggleMaxPower(gamepad1.xWasPressed());
        }
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

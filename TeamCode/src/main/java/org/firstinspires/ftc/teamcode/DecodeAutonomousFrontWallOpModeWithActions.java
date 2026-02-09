package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;

import java.lang.reflect.Field;

/**
 * This OpMode demonstrates:
 * -Running the OpMode in a loop (while(opModeIsActive()).
 * -Using a state machine to control actions to be performed during each iteration.
 * -A way to flexibly set a start Pose on a configured teleOp class, to its configured
 *  static field the teleOp class uses for its start Pose.
 */
@Autonomous(name = "Start from front wall")
public class DecodeAutonomousFrontWallOpModeWithActions extends LinearOpMode {
    /*
     * If the teleOp OpMod running after this OpMode has a static start Pose field you can
     * set with this OpMode's last Pose, set teleOpClass name to its class name,
     * and startingPoseFieldName to the static Pose field's name.
     */
    private final String teleOpClassName = "PedroPathTeleOp";
    private final String startingPoseFieldName = "startPose";
    private Field startingPoseField = null;

    private PedroPathTelemetry pedroTelemetry;
    private Follower follower;
    private final double ballPickupPower = .2;
    private double currentMaxPower, defaultMaxPower;
    private final double maxMaxPower = 1;
    private AllianceColor selectedColor = AllianceColor.RED;
    private BallSpikeLocation ballSpikeLocation = BallSpikeLocation.AUDIENCE_SIDE;
    private int pathState = 0;
    private RobotHardware robot;
    private AutonomousPathsFrontWall pedroPaths;
    private PathChain pathFromLaunchZoneToStartBallPickup;
    private PathChain pathFromStartBallPickupToEndBallPickup;
    private PathChain pathFromEndBallPickupToLaunchZone;
    private PathChain pathFromLaunchZoneToLeave;

    private String pedroMessage = "NO MESSAGE";

    @Override
    public void runOpMode() throws InterruptedException {
        initTeleOpStartPoseField();

        robot = new RobotHardware(this);

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);

        follower = pedroPathConfiguration.getFollower();

        defaultMaxPower = follower.getMaxPowerScaling();
        currentMaxPower = defaultMaxPower;
        initSetup();
        follower.setMaxPower(currentMaxPower);

        pedroPaths = createPedroPaths(selectedColor);
        setBallSpikeLocationPaths();
        pedroTelemetry = new PedroPathTelemetry(telemetry, follower, selectedColor);

        follower.setStartingPose(pedroPaths.startingPose());

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            performActions();
            pedroTelemetry.pathTelemetry(pedroMessage);
            telemetry.addLine();
            RobotHardware.SpinnerVelocities spinnerVelocities = robot.getSpinnerVelocities();
            telemetry.addData("Left Spinner Velocity", spinnerVelocities.left);
            telemetry.addData("Right Spinner Velocity", spinnerVelocities.right);
            telemetry.setAutoClear(false); //Add above telemetry to pedroTelemetry.
            telemetry.update();
            telemetry.setAutoClear(true);
        }

        setLastPose(follower.getPose());
    }

    private void performActions() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    pedroMessage = "Going from wall to launch zone";
                    follower.followPath(pedroPaths.pathFromWallToLaunchZone(), true);
                    pathState++;
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    robot.startShooterMotors(1);
                    pedroMessage = "Going from launch zone to ball pickup";
                    follower.followPath(pathFromLaunchZoneToStartBallPickup, true);
                    pathState++;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    robot.startBallPickup();
                    pedroMessage = "Picking up balls";
                    follower.followPath(pathFromStartBallPickupToEndBallPickup, ballPickupPower, true);
                    pathState++;
                }
            case 3:
                if (!follower.isBusy()) {
                    pedroMessage = "Going from ball pickup to launch zone";
                    follower.followPath(pathFromEndBallPickupToLaunchZone, ballPickupPower, true);
                    pathState++;
                }
            case 4:
                if (!follower.isBusy()) {
                    robot.startShooterMotors(1);
                    pedroMessage = "Parking";
                    follower.followPath(pathFromLaunchZoneToLeave, true);
                    pathState++;
                }
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

    /**
     * Depending on the passed selectedColor, instantiate and return either a
     * RedPedroPathsFrontWall or BluePedroPathsFrontWall.
     * Both implement the AutonomousPedroPathsFrontWall interface.
     * @param selectedColor RED or BLUE
     * @return AutonomousPedroPathsFrontWall
     */
    private AutonomousPathsFrontWall createPedroPaths(AllianceColor selectedColor) {
        if (selectedColor == AllianceColor.RED) {
            return new RedPedroPathsFrontWall(follower);
        }
        else {
            return new BluePedroPathsFrontWall(follower, new RedPedroPathsFrontWall(follower));
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
            telemetry.addLine("Press X to toggle max speed between " + defaultMaxPower + " and " + maxMaxPower);
            telemetry.addData(String.valueOf(currentMaxPower), "currently selected");
            telemetry.update();

            selectedColor = AllianceColor.INSTANCE.toggleColor(gamepad1.rightBumperWasPressed(), selectedColor);
            ballSpikeLocation = BallSpikeLocation.INSTANCE.toggleLocation(gamepad1.leftBumperWasPressed(), ballSpikeLocation);
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

    /**
     * Get reference to the (static) field in the teleOpClass that will be used to set its startPose.
     * Run these "heavy lifters" early so we can minimize the work happening in setLastPose().
     */
    private void initTeleOpStartPoseField() {
        try {
            Class<?> teleOpClass = Class.forName(teleOpClassName);
            startingPoseField = teleOpClass.getField(startingPoseFieldName);
        } catch (ClassNotFoundException e) {
            pedroTelemetry.pathTelemetry("Class " + teleOpClassName + " not found.");
        }
        catch (NoSuchFieldException e) {
            pedroTelemetry.pathTelemetry("Field " + startingPoseFieldName + " not found for class " + teleOpClassName);
        }
    }

    /**
     * Set the startPose of the teleOpClass.
     * Typically this is the last/current Pose of the follower.
     * @param pose Pose for setting teleOpClass startPose.
     */
    private void setLastPose(Pose pose) {
        if (startingPoseField != null) {
            try {
                startingPoseField.set(null, pose);
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
    }
}

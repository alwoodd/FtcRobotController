package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;

import java.util.HashMap;

@Autonomous(name = "Start from front wall")
public class DecodeAutonomousFrontWallOpModeWithActions extends LinearOpMode {
    private PedroPathTelemetry pedroTelemetry;
    private Follower follower;
    private final double ballPickupPower = .2;
    private double currentMaxPower, defaultMaxPower;
    private final double maxMaxPower = 1;
    private AllianceColor selectedColor = AllianceColor.RED;
    private int pathState = 0;
    private RobotHardware robot;
    private AutonomousPedroPathsFrontWall pedroPaths;
    private String pedroMessage = "NO MESSAGE";

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(this);

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);

        follower = pedroPathConfiguration.getFollower();

        defaultMaxPower = follower.getMaxPowerScaling();
        currentMaxPower = defaultMaxPower;
        initSetup();
        follower.setMaxPower(currentMaxPower);

        pedroPaths = createPedroPaths(selectedColor);
        pedroTelemetry = new PedroPathTelemetry(telemetry, follower, selectedColor);

        follower.setStartingPose(pedroPaths.startingPose());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
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
                    follower.followPath(pedroPaths.pathFromStartBallPickupToEndBallPickup(), true);
                    pathState++;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    robot.startBallPickup();
                    pedroMessage = "Picking up balls";
                    follower.followPath(pedroPaths.pathFromStartBallPickupToEndBallPickup(), ballPickupPower, true);
                    pathState++;
                }
            case 3:
                if (!follower.isBusy()) {
                    pedroMessage = "Going from ball pickup to launch zone";
                    follower.followPath(pedroPaths.pathFromEndBallPickupToLaunchZone(), ballPickupPower, true);
                    pathState++;
                }
            case 4:
                if (!follower.isBusy()) {
                    robot.startShooterMotors(1);
                    pedroMessage = "Parking";
                    follower.followPath(pedroPaths.pathFromLaunchZoneToPark(), true);
                    pathState++;
                }
        }
    }

    /**
     * Depending on the passed selectedColor, instantiate and return either a
     * RedPedroPathsFrontWall or BluePedroPathsFrontWall.
     * Both implement the AutonomousPedroPathsFrontWall interface.
     * @param selectedColor RED or BLUE
     * @return AutonomousPedroPathsFrontWall
     */
    private AutonomousPedroPathsFrontWall createPedroPaths(AllianceColor selectedColor) {
        if (selectedColor == AllianceColor.RED) {
            return new RedPedroPathsFrontWall(follower);
        }
        else {
            return new BluePedroPathsFrontWall(follower, new RedPedroPathsFrontWall(follower));
        }
    }

    /**
     * Call AllianceColor.INSTANCE.toggleColor() and toggleMaxPower()
     * as long as opModeInInit().
     */
    private void initSetup() {
        while (opModeInInit()) {
            telemetry.addLine("Press Right Bumper to toggle between Red and Blue alliance.");
            //telemetry.addLine("Press Right Bumper to confirm selection.");
            telemetry.addData(selectedColor.toString(), " currently selected");
            telemetry.addLine();
            telemetry.addLine("Press X to toggle max speed between " + defaultMaxPower + " and " + maxMaxPower);
            telemetry.addData(String.valueOf(currentMaxPower), "currently selected");
            telemetry.update();

            selectedColor = AllianceColor.INSTANCE.toggleColor(gamepad1.rightBumperWasPressed(), selectedColor);
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

package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathFollower;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;

@Autonomous(name = "Start from front wall")
public class DecodeAutonomousFrontWallOpMode extends LinearOpMode {
    private PedroPathTelemetry pedroTelemetry;
    private Follower follower;
    private final double ballPickupPower = .2;
    private double currentMaxPower, defaultMaxPower;
    private final double maxMaxPower = 1;
    private AllianceColor selectedColor = AllianceColor.RED;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);

        follower = pedroPathConfiguration.getFollower();

        defaultMaxPower = follower.getMaxPowerScaling();
        currentMaxPower = defaultMaxPower;
        initSetup();
        follower.setMaxPower(currentMaxPower);

        AutonomousPedroPathsFrontWall pedroPaths = createPedroPaths(selectedColor);
        pedroTelemetry = new PedroPathTelemetry(telemetry, follower, selectedColor);
        PedroPathFollower pedroPathFollower = new PedroPathFollower(this, follower, pedroTelemetry, pedroPaths.startingPose());

        waitForStart();

        if (opModeIsActive()) {
            pedroPathFollower.followPathChain(pedroPaths.pathFromWallToLaunchZone(), "Going from wall to launch zone");
            shootBalls();
            pedroPathFollower.followPathChain(pedroPaths.pathFromLaunchZoneToStartBallPickup(), "Going from launch zone to ball pickup");
            ballPickup();
            pedroPathFollower.followPathChain(pedroPaths.pathFromStartBallPickupToEndBallPickup(), ballPickupPower, "Picking up balls");
            pedroPathFollower.followPathChain(pedroPaths.pathFromEndBallPickupToLaunchZone(), "Going from ball pickup to launch zone");
            shootBalls();
            pedroPathFollower.followPathChain(pedroPaths.pathFromLaunchZoneToPark(), "Parking");
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

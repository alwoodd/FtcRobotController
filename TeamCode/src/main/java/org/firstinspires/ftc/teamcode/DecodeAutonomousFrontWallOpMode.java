package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathFollower;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;

@Autonomous(name = "Start from center wall")
public class DecodeAutonomousFrontWallOpMode extends LinearOpMode {
    private PedroPathTelemetry pedroTelemetry;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        PedroPathHardware pedroPathHardware = new PedroPathHardware(this);

        follower = pedroPathHardware.getFollower();

        AllianceColor selectedColor = selectRedOrBlue();
        AutonomousPedroPathsFrontWall pedroPaths = createPedroPaths(selectedColor);

        pedroTelemetry = new PedroPathTelemetry(telemetry, follower, selectedColor);
        PedroPathFollower pedroPathFollower = new PedroPathFollower(this, follower, pedroTelemetry, pedroPaths.startingPose());

        waitForStart();

        if (opModeIsActive()) {
            pedroPathFollower.followPathChain(pedroPaths.pathFromWallToLaunchZone(), "Going from wall to launch zone");
            shootBalls();
            pedroPathFollower.followPathChain(pedroPaths.pathFromLaunchZoneToBallPickup(), "Going from launch zone to ball pickup");
            ballPickup();
            pedroPathFollower.followPathChain(pedroPaths.pathFromBallPickupToLaunchZone(), "Going from ball pickup to launch zone");
            shootBalls();
//            pedroPathFollower.followPathChain(pedroPaths.pathFromBallPickupToLaunchZone());
//            shootBalls();
        }
    }

    private AutonomousPedroPathsFrontWall createPedroPaths(AllianceColor selectedColor) {
        if (selectedColor == AllianceColor.RED) {
            return new RedPedroPathsFrontWall(follower);
        }
        else {
            return new BluePedroPathsFrontWall(follower, new RedPedroPathsFrontWall(follower));
        }
    }

    private AllianceColor selectRedOrBlue() {
        AllianceColor selectedColor = AllianceColor.RED;

        while (!isStopRequested()) {
            telemetry.addLine("Press Left Bumper to toggle between Red and Blue alliance.");
            telemetry.addLine("Press Right Bumper to confirm selection.");
            telemetry.addData(selectedColor.toString(), " currently selected");
            telemetry.update();

            if (gamepad1.leftBumperWasPressed()) {
                if (selectedColor == AllianceColor.RED) {
                    selectedColor = AllianceColor.BLUE;
                }
                else {
                    selectedColor = AllianceColor.RED;
                }
            }
            else if (gamepad1.rightBumperWasPressed()) {
                break;
            }
        }

        return selectedColor;
    }

    private void shootBalls(){
        pedroTelemetry.pathTelemetry("Shooting balls.");
        sleep(4000);
    }

    private void ballPickup(){
        pedroTelemetry.pathTelemetry("Picking up balls.");
        sleep(4000);
    }
}

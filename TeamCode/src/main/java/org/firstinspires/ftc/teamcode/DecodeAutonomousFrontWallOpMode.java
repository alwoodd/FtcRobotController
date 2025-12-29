package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathFollower;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;

@Autonomous(name = "Red alliance start from center wall")
public class DecodeAutonomousFrontWallOpMode extends LinearOpMode {
    private PedroPathTelemetry pedroTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        PedroPathHardware pedroPathHardware = new PedroPathHardware(this);

        Follower follower = pedroPathHardware.getFollower();

        AutonomousPedroPathsFrontWall pedroPaths = new RedPedroPathsFrontWall(follower);

        pedroTelemetry = new PedroPathTelemetry(telemetry, follower);

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

    private void shootBalls(){
        pedroTelemetry.pathTelemetry("Shooting balls.");
        sleep(4000);
    }

    private void ballPickup(){
        pedroTelemetry.pathTelemetry("Picking up balls.");
        sleep(4000);
    }
}

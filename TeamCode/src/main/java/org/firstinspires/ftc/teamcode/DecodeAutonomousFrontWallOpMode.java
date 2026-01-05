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

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);

        follower = pedroPathConfiguration.getFollower();

        AllianceColor selectedColor = AllianceColor.INSTANCE.selectRedOrBlue(this);
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
        pedroTelemetry.pathTelemetry("Picking up balls.");
        sleep(4000);
    }
}

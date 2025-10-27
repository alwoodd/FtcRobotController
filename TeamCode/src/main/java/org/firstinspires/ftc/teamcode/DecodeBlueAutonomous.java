package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.teamPedroPathing.AutonomousPedroPaths;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConverter;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathFollower;

@Autonomous(name = "Blue alliance start from center wall")
public class DecodeBlueAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        PedroPathHardware pedroPathHardware = new PedroPathHardware(this);

        AutonomousPedroPaths pedroPaths = new BluePedroPathsFrontWall(pedroPathHardware.getFollower());

        PathChain initialPathChain = pedroPaths.pathFromWallToLaunchZone(); //Have first path ready to go.

        PedroPathFollower pedroPathFollower = new PedroPathFollower(this, pedroPathHardware.getFollower());

        waitForStart();

        if (opModeIsActive()) {
            pedroPathFollower.followPathChain(initialPathChain);
            putSpecimenOnChamber();
            pedroPathFollower.followPathChain(pedroPaths.pathFromChamberToSpike());
            pickupSpikeSample();
            //etc.
        }
    }

/*
    private void followPathChain(PathChain pathChain) {
        follower.followPath(pathChain);

        while (opModeIsActive() && !follower.isBusy()){
            follower.update();
        }
    }
*/

    private void putSpecimenOnChamber(){}

    private void pickupSpikeSample(){}
}

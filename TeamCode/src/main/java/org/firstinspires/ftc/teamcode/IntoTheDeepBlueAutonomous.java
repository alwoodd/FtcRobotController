package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.teamPedroPathing.AutonomousPedroPaths;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathUtil;

@Autonomous(name = "Blue alliance start from center wall")
public class IntoTheDeepBlueAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        Follower follower = new Follower(this.hardwareMap);
        PedroPathUtil pedroPathUtil = new PedroPathUtil(this, follower);

        AutonomousPedroPaths pedroPaths = new BluePedroPathsCenterWall();

        PathChain initialPathChain = pedroPaths.pathFromWallToChamber(); //Have first path ready to go.

        waitForStart();

        if (opModeIsActive()) {
            pedroPathUtil.followPathChain(initialPathChain);
            putSpecimenOnChamber();
            pedroPathUtil.followPathChain(pedroPaths.pathFromChamberToSpike());
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

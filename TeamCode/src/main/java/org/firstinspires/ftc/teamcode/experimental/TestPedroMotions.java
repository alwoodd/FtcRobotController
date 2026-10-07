package org.firstinspires.ftc.teamcode.experimental;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConfiguration;
import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;
import org.lhssa.ftc.teamcode.pedroPathing.PedroMotion;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPathData;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPathTelemetry;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPather;
import org.lhssa.ftc.teamcode.pedroPathing.PedroSleep;

@Autonomous
public class TestPedroMotions extends LinearOpMode {
    Pose startPose = new Pose(56, 8, Math.toRadians(90));
    Pose endPose1 = new Pose(56, 20, Math.toRadians(90));
    //Pose endPose2 = new Pose(68, 20, Math.toRadians(135));
    Pose endPose2 = new Pose(56, 20, Math.toRadians(135));

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = new PedroPathConfiguration(this).getFollower();
        follower.setPose(startPose);

        PedroPathTelemetry pedroPathTelemetry = new PedroPathTelemetry(telemetry, follower, AllianceColor.RED);

        PedroPather pather = new PedroPather(AllianceColor.RED, AllianceColor.RED);
        PedroMotion motion = new PedroMotion(follower);
        //PedroSleep pedroSleep = new PedroSleep(follower);

        PedroPathData[] paths = {pather.pathBetween(startPose, endPose1),
                    pather.pathBetween(endPose1, endPose2)
        };

        pedroPathTelemetry.pathTelemetry("Ready to start");
        waitForStart();

        int currentPathNum = 0;
        while(opModeIsActive()) {
            follower.update();
            motion.goPath(paths[currentPathNum], .3);
            pedroPathTelemetry.pathTelemetry("Driving path # " + currentPathNum);
            if (motion.isPathComplete()) {
//                pedroPathTelemetry.pathTelemetry(currentPathNum + " is Complete");
//                pedroSleep.sleep(800);
                currentPathNum++;
                if (currentPathNum > paths.length - 1) {
                    break;
                }
            }
        }

    }
}

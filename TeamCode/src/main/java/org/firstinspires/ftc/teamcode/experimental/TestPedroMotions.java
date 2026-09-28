package org.firstinspires.ftc.teamcode.experimental;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConfiguration;
import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;
import org.lhssa.ftc.teamcode.pedroPathing.PedroMotion;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPather;
import org.opencv.core.Mat;

@Autonomous
public class TestPedroMotions extends LinearOpMode {
    Pose startPose = new Pose(56, 8, Math.toRadians(90));
    Pose endPose1 = new Pose(56, 20, Math.toRadians(90));
    Pose endPose2 = new Pose(56, 20, Math.toRadians(135));

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = new PedroPathConfiguration(this).getFollower();
        follower.setPose(startPose);

        PedroPather pather = new PedroPather(AllianceColor.RED, AllianceColor.RED);
        PedroMotion motion = new PedroMotion(follower);

        Path path1 = pather.pathBetween(startPose, endPose1);
        Path path2 = pather.pathBetween(endPose1, endPose2);
        Path[] paths = {pather.pathBetween(startPose, endPose1),
                    pather.pathBetween(endPose1, endPose2)
        };

        waitForStart();

        int currentPathNum = 0;
        while(opModeIsActive()) {
            follower.update();
            motion.goPath(paths[currentPathNum]);
            if (motion.isPathComplete()) {
                currentPathNum++;
                if (currentPathNum > paths.length - 1) {
                    break;
                }
            }
        }

    }
}

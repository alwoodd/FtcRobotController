package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

public class PedroPathFollower {
    private final LinearOpMode opMode;
    private final Follower follower;

    public PedroPathFollower(LinearOpMode opMode, Follower follower) {
        this.opMode = opMode;
        this.follower = follower;
    }

    public void followPathChain(PathChain pathChain) {
        follower.followPath(pathChain);

        while (opMode.opModeIsActive() && !follower.isBusy()){
            follower.update();
        }
    }
}

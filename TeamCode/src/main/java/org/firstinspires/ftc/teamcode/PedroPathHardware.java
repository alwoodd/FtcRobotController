package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class PedroPathHardware {
    private final OpMode myOpMode;

    private Follower follower;

    public PedroPathHardware(LinearOpMode opMode) {
        this.myOpMode = opMode;
        init();
    }

    private void init() {
        this.follower = new FollowerBuilder(buildFollowerConstants(), myOpMode.hardwareMap)
                .build();
    }

    private FollowerConstants buildFollowerConstants() {
        FollowerConstants followConstants = new FollowerConstants();

        /*
         * All of these need to be set using your robot's tuning values!
         * There are loads of other properties in the FollowConstants class that are unfortunately not documented anywhere.
         */
        followConstants
            .mass(1.0)                          //Kilograms
            .forwardZeroPowerAcceleration(-1)   //https://pedropathing.com/docs/pathing/tuning/automatic#forward-zero-power-acceleration
            .lateralZeroPowerAcceleration(-1);  //https://pedropathing.com/docs/pathing/tuning/automatic#lateral-zero-power-acceleration

        return followConstants;
    }

    public Follower getFollower() {
        return follower;
    }
}

package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This class provides a single location to set Pedro Path's myriad constraints.
 * It is responsible for creating and returning a Follower built using these constraints.
 */
public class PedroPathConfiguration {
    private final OpMode myOpMode;

    private Follower follower;

    public PedroPathConfiguration(OpMode opMode) {
        this.myOpMode = opMode;
        init();
    }

    /**
     * Call all the constant builders and build the Follower instance.
     */
    private void init() {
        HardwareMap hwMap = myOpMode.hardwareMap;

        this.follower = new Follower(
                new PinpointLocalizer(hwMap, buildPinpointConstants()),
                new Mecanum(hwMap, buildMecanumConfig()),
                new Foresight(buildForesightConfig()));
    }

        private MecanumConfig buildMecanumConfig() {
        return new MecanumConfig(config -> {
            config.frontLeftName.set("leftFront");
            config.frontRightName.set("rightFront");
            config.backLeftName.set("leftRear");
            config.backRightName.set("rightRear");
            config.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            config.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            config.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            config.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        });
    }

    private PinpointConfig buildPinpointConstants() {
        return new PinpointConfig(config -> {
            config.name.set("pinpoint");
            config.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            config.xPodOffset.set(4.499384662297767);
            config.yPodOffset.set(-0.34130013833834427);
            config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            config.globalDistanceUnit.set(DistanceUnit.INCH);
            config.offsetUnits.set(DistanceUnit.INCH);
        });
    }

    private ForesightConfig buildForesightConfig() {
        return new ForesightConfig(config -> {
            Controller primaryTranslationalForward = Controller.proportional(0.15754912724332415);
            Controller secondaryTranslationalForward = Controller.proportional(0.05821019272619552);
            Controller primaryTranslationalLateral = Controller.proportional(0.2396363450486639);
            Controller secondaryTranslationalLateral = Controller.proportional(0.08853922629441217);

            config.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            config.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            config.coast.set(Controller.proportionalFeedforward(0.012037834786413674));
            config.brake.set(Controller.proportionalFeedforward(0.010232159568451624));

            config.headingFeedback.set(Controller.proportional(2.7041467678965883));
            config.headingBrakeCoefficients.set(Vector2D.cartesian(0.04680784239650283, 0.009565214030704722));

            config.linearBrakeCoefficients.set(Matrix.diag(0.009333182410668594, 0.049155086334919704));
            config.quadraticBrakeCoefficients.set(Matrix.diag(0.003294144693897478, 0.001875563853312929));

            config.maxAchievableForwardVelocity.set(80.68893150711817);
            config.maxAchievableStrafeVelocity.set(58.40755147416206);
            config.naturalForwardDeceleration.set(36.32395067075196);
            config.naturalStrafeDeceleration.set(56.44869265274182);

            config.maxPathSpeed.set(.6);
        });
    }

     public Follower getFollower() {
        return follower;
    }
}

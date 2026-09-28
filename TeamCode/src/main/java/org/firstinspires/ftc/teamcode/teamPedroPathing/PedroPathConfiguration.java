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
            config.xPodOffset.set(5.4375/*5.34638412355438*/);
            config.yPodOffset.set(0.0/*0.20986864885945958*/);
            config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            config.globalDistanceUnit.set(DistanceUnit.INCH);
            config.offsetUnits.set(DistanceUnit.INCH);
        });
    }

    private ForesightConfig buildForesightConfig() {
        return new ForesightConfig(config -> {
            Controller primaryTranslationalForward = Controller.proportional(0.23316259308933257);
            Controller secondaryTranslationalForward = Controller.proportional(0.08614734792727745);
            Controller primaryTranslationalLateral = Controller.proportional(0.3018797585217018);
            Controller secondaryTranslationalLateral = Controller.proportional(0.11153650439806052);

            config.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            config.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            config.coast.set(Controller.proportionalFeedforward(0.01225405719538536));
            config.brake.set(Controller.proportionalFeedforward(0.010415948616077555));

            config.headingFeedback.set(Controller.proportional(3.208995691632868));
            config.headingBrakeCoefficients.set(Vector2D.cartesian(0.05476520348664941, 0.013856353655144643));

            config.linearBrakeCoefficients.set(Matrix.diag(0.0961186735402374, 0.04661418459162977));
            config.quadraticBrakeCoefficients.set(Matrix.diag(0.0026173428496953, 0.0035180928894402433));

            config.maxAchievableForwardVelocity.set(75.76091484132641);
            config.maxAchievableStrafeVelocity.set(59.76861401581861);
            config.naturalForwardDeceleration.set(24.494434793868912);
            config.naturalStrafeDeceleration.set(30.686920911960016);

            config.maxPathSpeed.set(.6);
        });
    }

     public Follower getFollower() {
        return follower;
    }
}

package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.lhssa.ftc.teamcode.pedroPathing.SwyftMecanum;

/**
 * This class provides a single location to set Pedro Path's myriad constraints.
 * It is responsible for creating and returning a Follower built using these constraints.
 */
public class PedroPathConfiguration {
    private final OpMode myOpMode;

    private Follower follower;

    public PedroPathConfiguration(LinearOpMode opMode) {
        this.myOpMode = opMode;
        init();
    }

    /**
     * Call all the constant builders and build the Follower instance.
     */
    private void init() {
        MecanumConstants driveConstants = buildMecanumConstants();

        this.follower = new FollowerBuilder(buildFollowerConstants(), myOpMode.hardwareMap)
                .setDrivetrain(new SwyftMecanum(myOpMode.hardwareMap, driveConstants))
                .pinpointLocalizer(buildPinpointConstants())
                .pathConstraints(buildPathConstraints())
                .build();

        /*
         * Follower has its own globalMaxPower that is initialized to 1.
         * Irritatingly, this cannot be changed using FollowerConstants.
         * It can only be changed by calling setMaxPower() on the built instance
         * of Follower. Set globalMaxPower to be the the same as that in DriveConstants.
         * (DriveConstants are set via MecanumConstants.)
         */
        this.follower.setMaxPower(driveConstants.getMaxPower());
    }

    private FollowerConstants buildFollowerConstants() {
        FollowerConstants followerConstants = new FollowerConstants()
            .mass(4.452)                          //Kilograms
            .forwardZeroPowerAcceleration(-33)
            .lateralZeroPowerAcceleration(-48.24)
            .headingPIDFCoefficients(new PIDFCoefficients(.35, 0, .01, .024))
            .translationalPIDFCoefficients(new PIDFCoefficients(.025,0,0,.019))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0, 0, .6, .15))
            .centripetalScaling(.0004);

        //followerConstants.setTurnHeadingErrorThreshold(.05);
        followerConstants.setHoldPointHeadingScaling(.7);
        return followerConstants;
    }

    private MecanumConstants buildMecanumConstants() {
        return new MecanumConstants()
            .maxPower(.8)
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(false) //Affects DriveTrain.startTeleopDrive() overload.
            .xVelocity(22.9)
            .yVelocity(20.5);
    }

    private PinpointConstants buildPinpointConstants() {
        return new PinpointConstants()
            .forwardPodY(5.4375)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    private PathConstraints buildPathConstraints() {
        return new PathConstraints(.99, 100, .9, 1);
    }

    public Follower getFollower() {
        return follower;
    }
}

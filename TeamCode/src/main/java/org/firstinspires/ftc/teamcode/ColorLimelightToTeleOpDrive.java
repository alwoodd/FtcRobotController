package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConfiguration;

@Autonomous
public class ColorLimelightToTeleOpDrive extends OpMode {
    private final boolean IS_LOGGING = true;
    private final double KP_TURN = .015;
    private final double TURN_THRESHOLD_DEGREES = 2;
    private final double MAX_FORWARD_POWER = .9;
    private final double TARGET_POLLEN_TA = .9;
    private final int TA_STALL_THRESHOLD = 4;

    private LimeLightHardware llHardware;
    private Follower follower;
    private boolean isFollowing = false;
    private double targetPedroHeadingDegrees;
    private boolean wasTurning;
    private boolean isForwardDone;

    private static final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    @Override
    public void init() {
        llHardware = new LimeLightHardware(this, 9);
        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);
        follower = pedroPathConfiguration.getFollower();
        follower.setStartingPose(startPose);
        follower.setMaxPower(.3);
        targetPedroHeadingDegrees = Math.toDegrees(startPose.getHeading());
        wasTurning = false;
        isForwardDone = false;
    }

    @Override
    public void start() {
        llHardware.startLimelight();
        llHardware.beginSearch();
        follower.startTeleOpDrive(true);
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            isFollowing = !isFollowing;
        }

        follower.update();
        llHardware.update();

        double rawTx = llHardware.getRawTx();
        double smoothedTx = llHardware.getSmoothedTx();
        double rawTa = llHardware.getRawTa();
        double smoothedTa = llHardware.getSmoothedTa();

        telemetry.addData("raw Tx", rawTx);
        telemetry.addData("Smoothed Tx", smoothedTx);
        telemetry.addData("raw Ta", rawTa);
        telemetry.addData("Smoothed Ta", smoothedTa);
        telemetry.addData("TARGET_POLLEN_TA", TARGET_POLLEN_TA);
        double currentPedroHeadingDegrees = Math.toDegrees(follower.getHeading());

        /*
         * Turning power calculations
         */
        double turnPower = 0; //Negative turns right, positive left.
        /*
         * If the difference between the current heading and the target heading > TURN_THRESHOLD_DEGREES,
         * calculate a turn power using that difference * the KP_TURN factor.
         */
        if (Math.abs(currentPedroHeadingDegrees - targetPedroHeadingDegrees) > TURN_THRESHOLD_DEGREES) {
            turnPower = (targetPedroHeadingDegrees - currentPedroHeadingDegrees) * KP_TURN;
            wasTurning = true;
        }
        /*
         * If we're done turning, resetTx for the next turning calculation.
         */
        else if (wasTurning) {
            llHardware.resetTx();
            robotLog("resetResults called");
            wasTurning = false;
        }
        /*
         * If we weren't turning, compute a new target heading based on the latest smoothedTx.
         */
        else {
            targetPedroHeadingDegrees = LimeLightHardware.txToPedroHeadingDegrees(currentPedroHeadingDegrees, smoothedTx);
        }

        /*
         * Forward power calculations
         */
        double forwardPower = 0;
        /*
         * If the current smoothedTa < TARGET_POLLEN_TA (i.e., we're not close enough),
         * calculate a new forwardPower.
         */
        boolean isSmoothedTaStalled = llHardware.isSmoothedTaStalled(TA_STALL_THRESHOLD);
        if (smoothedTa >= TARGET_POLLEN_TA || isSmoothedTaStalled) {
            isForwardDone = true;
            llHardware.resetTa();
        }

        if (!isForwardDone) {
            forwardPower = llHardware.taToPedroPower(smoothedTa, TARGET_POLLEN_TA, MAX_FORWARD_POWER);
        }

        /****************************************************************************************/
        telemetry.addLine();
        telemetry.addData("Is Following?", isFollowing);

        telemetry.addLine();
        telemetry.addData("isForwardDone?", isForwardDone);
        telemetry.addData("isTaStalled?", isSmoothedTaStalled);

        telemetry.addLine();
        telemetry.addData("Current Pedro Heading", currentPedroHeadingDegrees);
        telemetry.addData("Target Pedro Heading", targetPedroHeadingDegrees);

        telemetry.addLine();
        telemetry.addData("forwardPower", forwardPower);
        telemetry.addData("turnPower", "%.4f (%s)", turnPower, (turnPower < 0? "right":"left"));

        if (isFollowing) {
            follower.setTeleOpDrive(forwardPower, 0, turnPower);
        }
        robotLog("smoothed Ta: %.8f,TARGET_POLLEN_TA: %.4f, MAX_FORWARD_POWER: %.2f, fowardPower: %.2f, isForwardDone? %b, isSmoothedTaStalled? %b",
                smoothedTa, TARGET_POLLEN_TA, MAX_FORWARD_POWER, forwardPower, isForwardDone, isSmoothedTaStalled);
    }

    @Override
    public void stop() {
        //llHardware.logTaSmoothList();
        llHardware.stopLimelight();
    }

    private void robotLog(String format, Object ...args) {
        if (IS_LOGGING)
            RobotLog.ii("limelight", format, args);
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
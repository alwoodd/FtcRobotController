package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConfiguration;

@Autonomous
public class ColorLimelightToTeleOpDrive extends OpMode {
    private final boolean IS_LOGGING = true;
    private final double KP_TURN = .015;
    private final double TURN_THRESHOLD_DEGREES = 2;

    private final double FORWARD_DISTANCE_THRESHOLD_CM = 24;
    private final double FORWARD_POWER_AGGRESIVENESS = 1.7;
    private final double DESIRED_FORWARD_POWER_AT_THRESHOLD = .1;

    private final double BRAKING_DURATION_MS = 100;
    private final double BRAKING_POWER = -.1;

    private LimelightHardware llHardware;
    private Follower follower;
    private boolean isFollowing = false;

    private double targetPedroHeadingDegrees;
    private boolean wasTurning;

    private boolean wasMovingForward;
    private Timer timer;
    private Pose targetStartingPedroPose;
    private double targetDistanceCM;
    private boolean isStopped;

    private static final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    @Override
    public void init() {
        llHardware = new LimelightHardware(this, 9);
        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);
        follower = pedroPathConfiguration.getFollower();
        follower.setStartingPose(startPose);
        follower.setMaxPower(.3);
        targetPedroHeadingDegrees = Math.toDegrees(startPose.getHeading());
        targetDistanceCM = 0;
        targetStartingPedroPose = startPose;
        wasTurning = false;
        wasMovingForward = false;
        isStopped = false;
        timer = new Timer();
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
/*
        double rawTa = llHardware.getRawTa();
        double smoothedTa = llHardware.getSmoothedTa();
*/
        double rawTy = llHardware.getRawTy();
        double smoothedTy = llHardware.getSmoothedTy();

        telemetry.addData("raw Tx", rawTx);
        telemetry.addData("Smoothed Tx", smoothedTx);
/*
        telemetry.addData("raw Ta", rawTa);
        telemetry.addData("Smoothed Ta", smoothedTa);
*/
//        telemetry.addData("TARGET_POLLEN_TA", TARGET_POLLEN_TA);
        telemetry.addData("raw Ty", rawTy);
        telemetry.addData("Smoothed Ty", smoothedTy);

        /*
         * Turning power calculations
         */
        double turnPower = 0; //Negative turns right, positive left.
        double currentPedroHeadingDegrees = Math.toDegrees(follower.getHeading());
        double angleRemaining = targetPedroHeadingDegrees - currentPedroHeadingDegrees;
        /*
         * If the difference between the current heading and the target heading > TURN_THRESHOLD_DEGREES,
         * calculate a turn power using that difference * the KP_TURN factor.
         */
        if (Math.abs(angleRemaining) > TURN_THRESHOLD_DEGREES) {
            turnPower = angleRemaining * KP_TURN;
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
            targetPedroHeadingDegrees = LimelightHardware.txToPedroHeadingDegrees(currentPedroHeadingDegrees, smoothedTx);
        }

        /*
         * Forward power calculations
         */
        double forwardPower = 0;
        double distanceTraveledCM = LimelightHardware.distanceBetweenPosesCM(targetStartingPedroPose, follower.getPose());
        double distanceRemainingCM = targetDistanceCM - distanceTraveledCM;
        /*
         * If the remaining distance is greater than DISTANCE_THRESHOLD_CM,
         * calculate a forward power using the remaining distance * the KP_FORWARD factor.
         */
        if (distanceRemainingCM  > FORWARD_DISTANCE_THRESHOLD_CM) {
            forwardPower = forwardPower(distanceRemainingCM);//distanceRemainingCM * KP_FORWARD;
            wasMovingForward = true;
        }
        /*
         * If we're done moving forward, resetTy for the next forward calculation.
         * Also reset the timer used when applying braking power.
         */
        else if (wasMovingForward) {
            llHardware.resetTy();
            timer.resetTimer();
            forwardPower = 0;
            wasMovingForward = false;
            isStopped = false;
        }
        /*
         * Apply some braking to make the robot stop more quickly.
         */
        else if (!isStopped) {
            if (timer.getElapsedTime() < BRAKING_DURATION_MS) {
                forwardPower = BRAKING_POWER;
            }
            else {
                forwardPower = 0;
                isStopped = true;
            }
        }
        /*
         * If we weren't moving forward or braking, compute a new target distance based on the latest smoothedTy.
         * Also set our target starting Pose to the current Pose.
         */
        else {
            //If smoothedTy is 0 (i.e., we don't see a target), the target distance is 0.
            targetDistanceCM = smoothedTy == 0 ? 0 : llHardware.distanceCM(smoothedTy);
            targetStartingPedroPose = follower.getPose();
        }
        /****************************************************************************************/
        telemetry.addLine();
        telemetry.addData("Is Following?", isFollowing);

        telemetry.addLine();
        telemetry.addData("Target distance (from LL)", targetDistanceCM);
        telemetry.addData("Distance remaining", distanceRemainingCM);
        telemetry.addData("Distance traveled", distanceTraveledCM);
        telemetry.addData("isStopped?", isStopped);

        telemetry.addLine();
        telemetry.addData("Current Pedro Heading", currentPedroHeadingDegrees);
        telemetry.addData("Target Pedro Heading", targetPedroHeadingDegrees);

        telemetry.addLine();
        telemetry.addData("forwardPower", forwardPower);
        telemetry.addData("turnPower", "%.4f (%s)", turnPower, (turnPower < 0? "right":"left"));

        telemetry.addLine();
        double distance = llHardware.distanceCM(smoothedTy);
        telemetry.addData("Distance using Ty", distance);

        if (isFollowing) {
            follower.setTeleOpDrive(forwardPower, 0, turnPower);
        }
        robotLog("Target Heading: %.2f, Current Heading: %.2f, Angle Remaining: %.2f, Turning Power: %.2f|Target Distance: %.2f, Distance Traveled: %.2f, Distance Remaining: %.2f, Forward Power: %.3f",
                targetPedroHeadingDegrees, currentPedroHeadingDegrees, angleRemaining, turnPower, targetDistanceCM, distanceTraveledCM, distanceRemainingCM, forwardPower);
    }

    @Override
    public void stop() {
        llHardware.stopLimelight();
    }

    private double forwardPower(double distanceRemaining) {
        double ratio = distanceRemaining / FORWARD_DISTANCE_THRESHOLD_CM;
        return (DESIRED_FORWARD_POWER_AT_THRESHOLD * Math.pow(ratio, FORWARD_POWER_AGGRESIVENESS));
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
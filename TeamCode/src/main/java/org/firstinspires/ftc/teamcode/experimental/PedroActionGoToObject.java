package org.firstinspires.ftc.teamcode.experimental;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.LimelightHardware;

/**
 * PedroAction that demonstrates using LimelightHardware info for calculating wheel powers.
 */
public class PedroActionGoToObject implements PedroAction{
    private final String description;
    private LimelightHardware llHardware;
    private final Follower follower;

    private final double KP_TURN = .015;
    private final double TURN_THRESHOLD_DEGREES = 2;
    private final double FORWARD_DISTANCE_THRESHOLD_CM = 24;
    private final double FORWARD_POWER_AGGRESIVENESS = 1.7;
    private final double DESIRED_FORWARD_POWER_AT_THRESHOLD = .1;

    private final double BRAKING_DURATION_MS = 100;
    private final double BRAKING_POWER = -.1;

    private double targetPedroHeadingDegrees;
    private boolean wasTurning;
    private Pose targetStartingPedroPose;
    private double targetDistanceCM;
    private boolean wasMovingForward;
    private final Timer timer;
    private boolean isStopped;

    /**
     * Constructor
     * @param description Specific action(s) being performed by this instance.
     * @param follower Follower instance
     * @param limelightHardware LimelightHardware instance
     */
    public PedroActionGoToObject(String description, Follower follower, LimelightHardware limelightHardware) {
        this.description = description;
        this.follower = follower;
        this.follower.startTeleOpDrive(true);
        this.llHardware = limelightHardware;
        this.llHardware.beginSearch();
        this.timer = new Timer();
    }

    @Override
    public void update() {
        double turnPower = calculateTurningPower();
        double forwardPower = calculateForwardPower();
        follower.setTeleOpDrive(forwardPower, 0, turnPower);
    }

    /**
     * isComplete is true if we're both done with turning and moving forward.
     * @return true if we're currently not moving towards a target object.
     */
    @Override
    public boolean isComplete() {
        return (!wasTurning && !wasMovingForward);
    }

    @Override
    public String getDescription() {
        return description;
    }

    private double calculateTurningPower() {
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
            wasTurning = false;
        }
        /*
         * If we weren't turning, compute a new target heading based on the latest smoothedTx.
         */
        else {
            targetPedroHeadingDegrees = LimelightHardware.txToPedroHeadingDegrees(currentPedroHeadingDegrees,
                llHardware.getSmoothedTx());
        }

        return turnPower;
    }

    private double calculateForwardPower() {
        double forwardPower = 0;
        double distanceTraveledCM = LimelightHardware.distanceBetweenPosesCM(targetStartingPedroPose, follower.getPose());
        double distanceRemainingCM = targetDistanceCM - distanceTraveledCM;
        /*
         * If the remaining distance is greater than DISTANCE_THRESHOLD_CM,
         * calculate a forward power using the remaining distance * the KP_FORWARD factor.
         */
        if (distanceRemainingCM  > FORWARD_DISTANCE_THRESHOLD_CM) {
            forwardPower = forwardPower(distanceRemainingCM);
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
            double smoothedTy = llHardware.getSmoothedTy();
            //If smoothedTy is 0 (i.e., we don't see a target), the target distance is 0.
            targetDistanceCM = smoothedTy == 0 ? 0 : llHardware.distanceCM(smoothedTy);
            targetStartingPedroPose = follower.getPose();
        }

        return forwardPower;
    }

    /**
     * Calculate a power based on distanceRemaining. The shorter the distance, the lower the power.
     * @param distanceRemaining distance remaining
     * @return power
     */
    private double forwardPower(double distanceRemaining) {
        double ratio = distanceRemaining / FORWARD_DISTANCE_THRESHOLD_CM;
        return (DESIRED_FORWARD_POWER_AT_THRESHOLD * Math.pow(ratio, FORWARD_POWER_AGGRESIVENESS));
    }
}

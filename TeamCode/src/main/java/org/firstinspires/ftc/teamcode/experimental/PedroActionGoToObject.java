package org.firstinspires.ftc.teamcode.experimental;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.LimelightHardware;

import org.lhssa.ftc.teamcode.pedroActions.PedroAction;

/**
 * PedroAction that demonstrates using LimelightHardware info for calculating wheel powers.
 */
public class PedroActionGoToObject implements PedroAction {
    private final String description;
    private LimelightHardware llHardware;
    private final Follower follower;

    private final double KP_TURN = .015;
    private final double TURN_THRESHOLD_DEGREES = 2;
    private final double FORWARD_DISTANCE_THRESHOLD_CM = 40;
    private final double FORWARD_POWER_AGGRESIVENESS = 2.5;
    private final double DESIRED_FORWARD_POWER_AT_THRESHOLD = .1;
    private final double MOVEMENT_DURATION_MS = 2000;

    private final double BRAKING_DURATION_MS = 200;
    private final double BRAKING_POWER = -.1;

    private double turnPower;
    private double forwardPower;
    private double targetPedroHeadingDegrees;
    private boolean wasTurning;
    private Pose targetStartingPedroPose;
    private double targetDistanceCM;
    private boolean wasMovingForward;
    private boolean isStopped;
    private Timer brakingTimer;
    private Timer movementTimer;
    private boolean isInitialized = false;

    /**
     * Constructor
     * @param description Specific action(s) being performed by this instance.
     * @param follower Follower instance
     * @param limelightHardware LimelightHardware instance
     */
    public PedroActionGoToObject(String description, Follower follower, LimelightHardware limelightHardware) {
        this.description = description;
        this.follower = follower;
        this.llHardware = limelightHardware;
    }

    @Override
    public void update() {
        if (!isInitialized) initialize();

        turnPower = calculateTurningPower();
        forwardPower = calculateForwardPower();
        follower.setTeleOpDrive(forwardPower, 0, turnPower);
    }

    /**
     * @return true if we haven't been moving for MOVEMENT_DURATION_MS.
     */
    @Override
    public boolean isComplete() {
        boolean isComplete = false;

        if (turnPower != 0 && forwardPower != 0) {
            movementTimer.resetTimer();
        }
        else if (movementTimer.getElapsedTime() > MOVEMENT_DURATION_MS) {
            isComplete = true;
            movementTimer.resetTimer();
        }

        return isComplete;
    }

    @Override
    public String getDescription() {
        return description;
    }

    /**
     * Initialize various values and settings.
     * This is intended to be called *once*.
     */
    private void initialize() {
        targetStartingPedroPose = follower.getPose();
        targetPedroHeadingDegrees = Math.toDegrees(follower.getHeading());
        targetDistanceCM = 0;
        wasTurning = false;
        wasMovingForward = false;
        isStopped = false;
        brakingTimer = new Timer();
        movementTimer = new Timer();
        llHardware.beginSearch();
        follower.startTeleOpDrive(true);

        isInitialized = true;
    }

    private double calculateTurningPower() {
        turnPower = 0; //Negative turns right, positive left.
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
        forwardPower = 0;
        double distanceTraveledCM = LimelightHardware.distanceBetweenPosesCM(targetStartingPedroPose, follower.getPose());
        double distanceRemainingCM = targetDistanceCM - distanceTraveledCM;
        /*
         * If the remaining distance is greater than DISTANCE_THRESHOLD_CM,
         * calculate a forward power using the remaining distance.
         */
        if (distanceRemainingCM > FORWARD_DISTANCE_THRESHOLD_CM) {
            forwardPower = forwardPower(distanceRemainingCM);
    //RobotLog.ii("Forward Power", "Distance remaining: %.2f; Forward power: %.2f", distanceRemainingCM, forwardPower);

            wasMovingForward = true;
        }
        /*
         * If we're done moving forward, resetTy for the next forward calculation.
         * Also reset the timer used when applying braking power.
         */
        else if (wasMovingForward) {
            llHardware.resetTy();
            brakingTimer.resetTimer();
            forwardPower = 0;
            wasMovingForward = false;
            isStopped = false;
        }
        /*
         * Apply some braking to make the robot stop more quickly.
         */
        else if (!isStopped) {
            if (brakingTimer.getElapsedTime() < BRAKING_DURATION_MS) {
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
     * Calculate a power based on distanceRemainingCM. The shorter the distance, the lower the power.
     * @param distanceRemainingCM distance remaining
     * @return power
     */
    private double forwardPower(double distanceRemainingCM) {
        double ratio = distanceRemainingCM / FORWARD_DISTANCE_THRESHOLD_CM;
        return (DESIRED_FORWARD_POWER_AT_THRESHOLD * Math.pow(ratio, FORWARD_POWER_AGGRESIVENESS));
    }
}

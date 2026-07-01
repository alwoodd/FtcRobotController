package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

public class LimelightHardware {
    private final double SMOOTHING_FACTOR = .1;
    private final Limelight3A limelight;
    private boolean isSearching = false;
    private final Smoothifier tXsmoothifier;
    private final Smoothifier tYsmoothifier;
    private final Smoothifier tAsmoothifier;
    private final List<Double> taSmoothList;

    private double rawTy;
    private double smoothedTy;
    private double rawTx;
    private double smoothedTx;
    private double rawTa;
    private double smoothedTa;

    private final TyDistancesTable tyDistancesTable;
    private final TyDistancesTable.TyDistance[] tyDistances = {
            new TyDistancesTable.TyDistance(-.999, 120),
            new TyDistancesTable.TyDistance(-3.2, 100),
            new TyDistancesTable.TyDistance(-6.6, 80),
            new TyDistancesTable.TyDistance(-11.8, 60),
            new TyDistancesTable.TyDistance(-19, 40)
        };

    /**
     * Manages Limelight hardware, and provides methods for OpMode use.
     * @param opMode Instance of OpMode
     * @param pipeLineNumber Initial Limelight pipeline number.
     */
    public LimelightHardware(OpMode opMode, int pipeLineNumber/*, double minPower, double maxPower*/) {
        //this.opMode = opMode;
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        setPipeLineNumber(pipeLineNumber);

        tXsmoothifier = new Smoothifier(SMOOTHING_FACTOR);
        tAsmoothifier = new Smoothifier(SMOOTHING_FACTOR);
        tYsmoothifier = new Smoothifier(SMOOTHING_FACTOR);

        taSmoothList = new ArrayList<>();

        tyDistancesTable = new TyDistancesTable();
        tyDistancesTable.addTyDistances(tyDistances);
    }

    /**
     * Enable Limelight result processing.
     */
    public void beginSearch() {
        isSearching = true;
    }

    /**
     * Disable Limelight result processing.
     */
    public void endSearch() {
        isSearching = false;
    }

    /**
     * This needs to be called during every iteration of the OpMode's processing loop.
     */
    public void update() {
        if (isSearching) {
            LLResult llResult = limelight.getLatestResult();

            if (llResult.isValid()) {
                rawTx = llResult.getTx();
                rawTa = llResult.getTa();
                rawTy = llResult.getTy();
                if (rawTy != 0) {
                    smoothedTy = tYsmoothifier.smooth(rawTy);
                }
                if (rawTx != 0) {
                    smoothedTx = tXsmoothifier.smooth(rawTx);
                }
                if (rawTa != 0) {
                    smoothedTa = tAsmoothifier.smooth(rawTa);
                    taSmoothList.add(smoothedTa);
                }
            }
        }
    }

    /**
     * @return Returns the latest valid value of llResult.getTx().
     */
    public double getRawTx() {
        return rawTx;
    }

    /**
     * @return Returns the latest valid value of llResult.getTa().
     */
    public double getRawTa() {
        return rawTa;
    }

    /**
     * @return Returns the latest valid value of llResult.getTy().
     */
    public double getRawTy() {
        return rawTy;
    }

    /**
     * @return Returns the smoothed Tx value based on the last valid raw Tx.
     */
    public double getSmoothedTx() {
        return smoothedTx;
    }

    /**
     * @return Returns the smoothed Ta value based on the last valid raw Ta.
     */
    public double getSmoothedTa() {
        return smoothedTa;
    }

    /**
     * @return Returns the smoothed Ta value based on the last valid raw Ta.
     */
    public double getSmoothedTy() {
        return smoothedTy;
    }

    /**
     * Call to reset all Tx-related values to 0.
     * Call, for example, if you want to start getting new smoothed values after
     * having finished current robot movement.
     */
    public void resetTx() {
        rawTx = 0;
        smoothedTx = 0;
        tXsmoothifier.reset();
    }

    /**
     * Call to reset all Ta-related values to 0.
     * Call, for example, if you want to start getting new smoothed values based on results after
     * having finished current robot movement.
     */
    public void resetTa() {
        rawTa = 0;
        smoothedTa = 0;
        tAsmoothifier.reset();
        taSmoothList.clear();
    }

    /**
     * Call to reset all TY-related values to 0.
     * Call, for example, if you want to start getting new smoothed values after
     * having finished current robot movement.
     */
    public void resetTy() {
        rawTy = 0;
        smoothedTy = 0;
        tAsmoothifier.reset();
    }

    /**
     * Convenience method to calculate a new heading offset by the passed Tx value.
     * @param oldHeading Old or Current heading
     * @param tX Degrees to adjust by.
     * @return new heading value.
     */
    public static double txToPedroHeadingDegrees(double oldHeading, double tX) {
        return oldHeading - tX;
    }

    /**
     * Calculate the distance between two Poses.
     * @param startPose Pose
     * @param endPose   Pose
     * @return distance in CM
     */
    public static double distanceBetweenPosesCM(Pose startPose, Pose endPose) {
        double distanceX = endPose.getX() - startPose.getX();
        double distanceY = endPose.getY() - startPose.getY();

        //Distance in inches
        double distance = Math.sqrt((distanceX * distanceX) + (distanceY * distanceY));
        //Distance in centimeters
        return (distance * 2.54);
    }

    /**
     * Calculate distance based on passed Ta angle.
     * @param tA Ta angle
     * @return results of TyDistancesTble.distanceFor()
     */
    public double distanceCM(double tA) {
        return tyDistancesTable.distanceFor(tA);
    }

    /**
     * Call to update the Limelight's pipeline number.
     * @param pipeline New Limelight pipeline number.
     */
    public void setPipeLineNumber(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    /**
     * Start the Limelight
     */
    public void startLimelight() {
        limelight.start();
    }

    /**
     * Stop the Limelight
     */
    public void stopLimelight() {
        endSearch();
        limelight.stop();
    }

    /**
     * Encapsulates a smoothing algorithm for successive values of T-values.
     */
    private static class Smoothifier {
        private final double alpha;
        private double prevSmoothed;
        private boolean initPrevSmoothed = true;

        /**
         * Ctor
         * @param alpha Alpha value used for smoothing algorithm.
         */
        public Smoothifier(double alpha) {
            this.alpha = alpha;
        }

        /**
         * Algorithm to smooth out passed angle values based on the prior smoothed value.
         * @param rawValue value to be smoothed
         * @return smoothed value
         */
        public double smooth(double rawValue) {
            if (initPrevSmoothed) {
                prevSmoothed = rawValue;
                initPrevSmoothed = false;
            }
            double smoothValue = (this.alpha * rawValue) + (1 - this.alpha) * prevSmoothed;
//            RobotLog.ii("Smoothifier", "rawValue: %.2f, prevSmoothed: %.2f, smoothValue: %.2f",
//                    rawValue, prevSmoothed, smoothValue);
            prevSmoothed = smoothValue;

            return smoothValue;
        }

        /**
         * Call to force smoothing to start over.
         */
        public void reset() {
            initPrevSmoothed = true;
        }
    }
}
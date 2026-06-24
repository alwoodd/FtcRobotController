package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

public class LimeLightHardware {
    //private final OpMode opMode;
    private final Limelight3A limelight;
    private boolean isSearching = false;
    private final double SMOOTHING_FACTOR = .01;
    private final Smoothifier tXsmoothifier;
    private final Smoothifier tAsmoothifier;
    private final List<Double> taSmoothList;

    private double rawTx;
    private double smoothedTx;
    private double rawTa;
    private double smoothedTa;
    private int smoothedTaMissCount;

    /**
     * Manages Limelight hardware, and provides methods for OpMode use.
     * @param opMode Instance of OpMode
     * @param pipeLineNumber Initial Limelight pipeline number.
     */
    public LimeLightHardware(OpMode opMode, int pipeLineNumber/*, double minPower, double maxPower*/) {
        //this.opMode = opMode;
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        setPipeLineNumber(pipeLineNumber);

        tXsmoothifier = new Smoothifier(SMOOTHING_FACTOR);
        tAsmoothifier = new Smoothifier(SMOOTHING_FACTOR);

        taSmoothList = new ArrayList<>();
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
            boolean isSmoothedTaMiss = true;

            if (llResult.isValid()) {
                rawTx = llResult.getTx();
                rawTa = llResult.getTa();
                if (rawTx != 0) {
                    smoothedTx = tXsmoothifier.smooth(rawTx);
                }
                if (rawTa != 0) {
                    smoothedTa = tAsmoothifier.smooth(rawTa);
                    isSmoothedTaMiss = false;
                    taSmoothList.add(smoothedTa);
                }
            }
            /*
             * If smoothedTa didn't get updated (isSmoothedTaMiss is true),
             * increment smoothedTaMissCount. Otherwise, set it to zero.
             */
            smoothedTaMissCount = isSmoothedTaMiss ? smoothedTaMissCount + 1 : 0;
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
     * Call to reset all Tx-related values to 0.
     * Call, for example, if you want to start getting new smoothed values based on results after
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
        smoothedTaMissCount = 0;
        tAsmoothifier.reset();
        taSmoothList.clear();
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
/*
    public void logTaSmoothList() {
*//*
        for (double ta : taSmoothList) {
            RobotLog.ii("smooth dump", "%.4f", ta);
        }
*//*
        for (int i = 0; i < taSmoothList.size(); i++) {
            RobotLog.ii("smooth dump", "%.4f", taSmoothList.get(i));
        }
    }*/

    /**
     * @param stallThreshold Desired threshold at which the current smoothedTa value
     *                       is considered stalled or unchanging.
     * @return true if smoothedTaMissCount > the passed smoothedTaMissCount
     */
    public boolean isSmoothedTaStalled(int stallThreshold) {
        return (smoothedTaMissCount >= stallThreshold);
    }

    /**
     * Calculates motor power based on how far apart tA and targetTa are.
     * The closer they are, the lower the calculated power value.
     * @param tA Current Ta. Typically a smoothed Ta.
     * @param targetTa Target Ta we're trying to reach.
     * @param maxPower Maximum power that can be returned.
     * @return power
     */
    public double taToPedroPower(double tA, double targetTa, double maxPower) {
        double currentMinTa = minSmoothedTa();
        double taRange = targetTa - currentMinTa;

        double distanceCovered = tA - currentMinTa;
        double progress = distanceCovered / taRange;
//        RobotLog.ii("taToPedroPower", "minPlusMaxPower: %.2f, tA: %.4f, currentMinTa: %.4f, currentMaxTa: %.4f",
//                minPlusMaxPower, tA, currentMinTa, currentMaxTa);

        return maxPower - progress; //power
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
     * @return Return the smallest taSmooth value encountered this instance
     * of LimeLightHardware, or since the last resetTa().
     */
    private double minSmoothedTa() {
        if (taSmoothList.isEmpty()) {
            return 0;
        }

        double minSmoothed = taSmoothList.get(0);

        for (double value : taSmoothList) {
            if (value < minSmoothed) {
                minSmoothed = value;
            }
        }

        return minSmoothed;
    }

    /**
     * Encapsulates a smoothing algorithm for successive values of Tx or Ty.
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
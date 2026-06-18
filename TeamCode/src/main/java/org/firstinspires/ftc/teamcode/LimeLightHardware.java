package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.RobotLog;

public class LimeLightHardware {
    private final Limelight3A limelight;
    private boolean isSearching = false;
    private final double SMOOTHING_FACTOR = .01;
    private final Smoothifier tXsmoothifier;
    private double rawTx;
    private double smoothedTx;

    /**
     * Manages Limelight hardware, and provides methods for OpMode use.
     * @param opMode Instance of OpMode
     * @param pipeLineNumber Initial Limelight pipeline number.
     */
    public LimeLightHardware(OpMode opMode, int pipeLineNumber) {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        setPipeLineNumber(pipeLineNumber);

        tXsmoothifier = new Smoothifier(SMOOTHING_FACTOR);
    }

    /**
     * Enable Limelight result processing.
     */
    public void beginSearch() {
        isSearching = true;
    }

    /**
     * Disable Limelight result processing.
     */    public void endSearch() {
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
                if (rawTx != 0) {
                    smoothedTx = tXsmoothifier.smooth(rawTx);
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
     * @return Returns the smoothed Tx value based on the last valid raw Tx.
     */
    public double getSmoothedTx() {
        return smoothedTx;
    }

    /**
     * Call to reset all values to 0.
     * Call, for example, if you want to start getting new smoothed values based on results after
     * having finished current robot movement.
     */
    public void resetResults() {
        rawTx = 0;
        smoothedTx = 0;
        tXsmoothifier.reset();
    }

    /**
     * Encapsulates a smoothing algorithm for successive values of Tx or Ty.
     */
    private static class Smoothifier {
        private final double alpha;
        private double prevSmoothed = 0;

        /**
         * Ctor
         * @param alpha Alpha value used for smoothing algorithm.
         */
        public Smoothifier(double alpha) {
            this.alpha = alpha;
        }

        /**
         * Algorithm to smooth out passed angle values based on the prior smoothed value.
         * @param angleDegrees angle to be smoothed
         * @return smoothed angle
         */
        public double smooth(double angleDegrees) {
            if (prevSmoothed == 0) {
                prevSmoothed = angleDegrees;
            }
            double smoothValue = (this.alpha * angleDegrees) + (1 - this.alpha) * prevSmoothed;
//            RobotLog.ii("Smoothifier", "angleDegrees: %.2f, prevSmoothed: %.2f, smoothValue: %.2f",
//                    angleDegrees, prevSmoothed, smoothValue);
            prevSmoothed = smoothValue;

            return smoothValue;
        }

        /**
         * Call to force smoothing to start over.
         */
        public void reset() {
            prevSmoothed = 0;
        }
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
}
package org.firstinspires.ftc.teamcode;

public enum FrontBackLocation {
    FRONT,
    BACK,
    INSTANCE;

    /**
     * Standard way to toggle the current spike location.
     * @param buttonPressed button result passed by the caller
     * @param currentLocation what the caller considers the current location
     * @return BallSpikeLocation AUDIENCE_SIDE, MIDDLE, or GOAL_SIDE
     */
    public FrontBackLocation toggleLocation(boolean buttonPressed, FrontBackLocation currentLocation) {
        if (buttonPressed) {
            if (currentLocation == FrontBackLocation.FRONT) {
                currentLocation = FrontBackLocation.BACK;
            }
            else {
                currentLocation = FrontBackLocation.FRONT;
            }
        }

        return currentLocation;
    }
}

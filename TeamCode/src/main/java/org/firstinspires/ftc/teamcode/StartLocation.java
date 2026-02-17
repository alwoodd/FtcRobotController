package org.firstinspires.ftc.teamcode;

public enum StartLocation {
    FRONT,
    BACK,
    INSTANCE;

    /**
     * Standard way to toggle the current spike location.
     * @param buttonPressed button result passed by the caller
     * @param currentLocation what the caller considers the current location
     * @return BallSpikeLocation AUDIENCE_SIDE, MIDDLE, or GOAL_SIDE
     */
    public StartLocation toggleLocation(boolean buttonPressed, StartLocation currentLocation) {
        if (buttonPressed) {
            if (currentLocation == StartLocation.FRONT) {
                currentLocation = StartLocation.BACK;
            }
            else {
                currentLocation = StartLocation.FRONT;
            }
        }

        return currentLocation;
    }
}

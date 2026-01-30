package org.firstinspires.ftc.teamcode;

public enum BallSpikeLocation {
    AUDIENCE_SIDE,
    MIDDLE,
    GOAL_SIDE,
    INSTANCE;

    /**
     * Standard way to toggle the current spike location.
     * @param buttonPressed button result passed by the caller
     * @param currentLocation what the caller considers the current location
     * @return BallSpikeLocation AUDIENCE_SIDE, MIDDLE, or GOAL_SIDE
     */
    public BallSpikeLocation toggleLocation(boolean buttonPressed, BallSpikeLocation currentLocation) {
        if (buttonPressed) {
            switch (currentLocation) {
                case AUDIENCE_SIDE:
                    currentLocation = MIDDLE;
                    break;
                case MIDDLE:
                    currentLocation = GOAL_SIDE;
                    break;
                case GOAL_SIDE:
                    currentLocation = AUDIENCE_SIDE;
                    break;
            }
        }

        return currentLocation;
    }
}

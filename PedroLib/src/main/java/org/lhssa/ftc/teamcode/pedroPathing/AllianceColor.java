package org.lhssa.ftc.teamcode.pedroPathing;

public enum AllianceColor {
    RED,
    BLUE,
    INSTANCE;

    /**
     * Standard way to toggle the current color.
     * @param buttonPressed button result passed by the caller
     * @param currentColor what the caller considers the current color
     * @return AllianceColor RED or BLUE
     */
    public AllianceColor toggleColor(boolean buttonPressed, AllianceColor currentColor) {
        if (buttonPressed) {
            if (currentColor == AllianceColor.RED) {
                currentColor = AllianceColor.BLUE;
            }
            else {
                currentColor = AllianceColor.RED;
            }
        }

        return currentColor;
    }
}

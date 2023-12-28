package org.firstinspires.ftc.teamcode;

/**
 * Class to reference red, green, blue, and alpha values.
 * Variable instances of this type can be used to hold the values from the
 * red(), green(), blue(), and alpha() methods of a ColorSensor instance.
 */
public class RGBAcolors {
    private int red;
    private int green;
    private int blue;
    private int alpha;
    public RGBAcolors(int red, int green, int blue, int alpha) {
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.alpha = alpha;
    }
    public int getRed() {
        return red;
    }

    public int getGreen() {
        return green;
    }

    public int getBlue() {
        return blue;
    }

    public int getAlpha() {
        return alpha;
    }
}

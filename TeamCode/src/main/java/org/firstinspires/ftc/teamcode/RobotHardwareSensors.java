package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DistanceSensor.distanceOutOfRange;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class RobotHardwareSensors {
    public static final double DEFAULT_APPROACH_SPEED = .4;
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    private DistanceSensor centerDistanceSensor;
    private DistanceSensor sideDistanceSensor;
    private ColorSensor colorSensor;

    /**
     * The one and only constructor requires a reference to an OpMode.
     * @param opmode
     */
    public RobotHardwareSensors(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Call init() to initialize all the robot's hardware.
     */
    public void init() {
        initDistanceSensors();
        initColorSensor();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
    /**
     * Initialize distance sensor(s).
     */
    private void initDistanceSensors() {
        centerDistanceSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "centerDistanceSensor");
        sideDistanceSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "sideDistanceSensor");
    }

    private void initColorSensor() {
        colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    /**
     * Set the RunMode for all wheel motors to the passed runMode.
     * @param runMode
     */
    public void setRunModeForAllWheels(DcMotor.RunMode runMode) {
        //...
    }

    /**
     * Set the Power for all wheels to the passed speed.
     * @param speed
     */
    public void setPowerAllWheels(double speed) {
        //...
    }

    /**
     * Return distance detected by Center Sensor in CM.
     * @return distance in CM
     */
    public double getCenterSensorDistanceInCM() {
        return centerDistanceSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Return distance detected by Center Sensor,
     * using whatever DistanceUnit is passed in.
     * Available units:
     *    DistanceUnit.MM
     *    DistanceUnit.CM
     *    DistanceUnit.METER
     *    DistanceUnit.INCH
     * @param distanceUnit
     * @return
     */
    public double getCenterSensorDistance(DistanceUnit distanceUnit) {
        return centerDistanceSensor.getDistance(distanceUnit);
    }

    /**
     * Return distance detected by Side Sensor in CM.
     * @return distance in CM
     */
    public double getSideSensorDistanceInCM() {
        return sideDistanceSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Return distance detected by Side Sensor,
     * using whatever DistanceUnit is passed in.
     * Available units:
     *    DistanceUnit.MM
     *    DistanceUnit.CM
     *    DistanceUnit.METER
     *    DistanceUnit.INCH
     * @param distanceUnit
     * @return
     */
    public double getSideSensorDistance(DistanceUnit distanceUnit) {
        return sideDistanceSensor.getDistance(distanceUnit);
    }

    /**
     * Return all color values from Color Sensor.
     * @return RGBAcolor
     */
    public RGBAcolors getSensorColors() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        int alpha = colorSensor.alpha();

        return new RGBAcolors(red, green, blue, alpha);
    }

    /**
     * Drive robot until the passed color is detected.
     * @param color
     * @param colorThreshold
     */
    public void driveToSpike(SpikeColor color, int colorThreshold, double approachSpeed) {
        RGBAcolors colors;

        setRunModeForAllWheels(DcMotor.RunMode.RUN_USING_ENCODER);
        setPowerAllWheels(approachSpeed);

        while (myOpMode.opModeIsActive()) {
            colors = getSensorColors();
            if (color == SpikeColor.RED && colors.getRed() > colorThreshold) {
                break;
            }
            else if (colors.getBlue() > colorThreshold) {
                break;
            }
        }

        setPowerAllWheels(0);
    }

    /**
     * Drive robot until the passed color is detected,
     * using DEFAULT_APPROACH_SPEED.
     * @param color
     * @param colorThreshold
     */
    public void driveToSpike(SpikeColor color, int colorThreshold) {
        driveToSpike(color, colorThreshold, DEFAULT_APPROACH_SPEED);
    }

    /**
     * Determine which position (1, 2, or 3) the sensor detects an object (such as a cube) is in.
     * In this example, if neither the center or side sensors detect an object,
     * the position is 3. If the center sensor detects an object, the position is 1. Lastly,
     * if the side sensor detects an object, the position is 2.
     *
     * @return int positionNumber
     */
    public int getSpikeObjectPosition() {
        double centerSensorDistance = getCenterSensorDistanceInCM();
        double sideSensorDistance = getSideSensorDistanceInCM();
        int positionNumber = 0;

        if (centerSensorDistance == distanceOutOfRange && sideSensorDistance == distanceOutOfRange) {
            myOpMode.telemetry.addData("NOT DETECTED", "Object not detected by any sensor!");
            positionNumber = 3;
        }
        else if (centerSensorDistance != distanceOutOfRange) {
            myOpMode.telemetry.addData("DETECTED SIDE", "Object distance is %.0f CM", sideSensorDistance);
            positionNumber = 1;
        }
        else {
            myOpMode.telemetry.addData("DETECTED CENTER", "Object distance is %.0f CM", centerSensorDistance);
            positionNumber = 2;
        }
        myOpMode.telemetry.update();

        return positionNumber;
    }
}

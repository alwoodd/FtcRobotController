package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardwarePotentiometer {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define all the HardwareDevices (Motors, Servos, etc.). Make them private so they can't be accessed externally.
    private DcMotor leftArm;
    private DcMotor rightArm;
    private AnalogInput potentiometer;

    // Hardware device constants.  Make them public so they can be used by the calling OpMode, if needed.
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double MAX_POTENTIOMETER_ANGLE = 270;

    /**
     * You can set the arm positions using angles and/or potentiometer voltage.
     * Tune these values for your robot's actual values.
     */
    public static final double ARM_PARKED_ANGLE = 0;
    public static final double ARM_PIXEL_PICKUP_ANGLE = 200;
    public static final double ARM_BACKDROP_ANGLE = 100;
    public static final double ARM_PARKED_VOLTAGE = 0;
    public static final double ARM_PIXEL_PICKUP_VOLTAGE = 3;
    public static final double ARM_BACKDROP_VOLTAGE = 1.4;

    /**
     * The one and only constructor requires a reference to an OpMode.
     * @param opmode
     */
    public RobotHardwarePotentiometer(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Call init() to initialize all the robot's hardware.
     */
    public void init() {
        initAnalogInputs();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    private void initAnalogInputs() {
        potentiometer = myOpMode.hardwareMap.get(AnalogInput.class, "potentiometer");
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    public void setArmPower(double power) {
        leftArm.setPower(power);
        rightArm.setPower(power);
    }

    public void setRunModeForAllArms(DcMotor.RunMode runMode) {
        leftArm.setMode(runMode);
        rightArm.setMode(runMode);
    }

    /**
     * Return potentiometer's current voltage.
     * @return voltage
     */
    public double getPotentiometerVoltage() {
        return potentiometer.getVoltage();
    }

    /**
     * Return calculated angle corresponding to a passed voltage.
     * Example: If the potentiometer's maximum voltage is 3.3,
     *          and it's maximum angle is 270 degrees,
     *          and the passed voltage is 1.65
     *          then the angle is 1.65 * 270 / 3.3 = 135 degrees.
     * @return angle
     */
    public double getPotentiometerAngle(double voltage ) {
        return voltage * MAX_POTENTIOMETER_ANGLE / potentiometer.getMaxVoltage();
    }

    /**
     * Return calculated angle corresponding to potentiometer's current voltage.
     * Example: If the potentiometer's maximum voltage is 3.3,
     *          and it's maximum angle is 270 degrees,
     *          and the current voltage is 1.65
     *          then the current angle is 1.65 * 270 / 3.3 = 135 degrees.
     * @return angle
     */
    public double getPotentiometerAngle() {
        return getPotentiometerAngle(potentiometer.getVoltage());
    }

    /**
     * Move arm up or down until it gets to potentiometer's targetVoltage.
     * @param targetVoltage
     */
    public void setArmPositionUsingVoltage(double targetVoltage) {
        double currentVoltage = potentiometer.getVoltage();
        setRunModeForAllArms(DcMotor.RunMode.RUN_USING_ENCODER);

        //Ex. If currentVoltage is 1, and targetVoltage is 2, we want to move arm up.
        //    If currentVoltage is 2, and targetVoltage is 1, we want to move arm down.
        if (currentVoltage < targetVoltage) {
            Telemetry.Item currentTelemetryItem = setupArmPositionTelemetry("Target Voltage", targetVoltage, "Current Voltage", currentVoltage);
            setArmPower(ARM_UP_POWER);

            while (myOpMode.opModeIsActive() && potentiometer.getVoltage() < targetVoltage) {
                currentTelemetryItem.setValue(potentiometer.getVoltage());
                myOpMode.telemetry.update();
            }

            setArmPower(0); //Whoa
        }
        else if (currentVoltage > targetVoltage) {
            Telemetry.Item currentTelemetryItem =  setupArmPositionTelemetry("Target Voltage", targetVoltage, "Current Voltage", currentVoltage);
            setArmPower(ARM_DOWN_POWER);

            while(myOpMode.opModeIsActive() && potentiometer.getVoltage() > targetVoltage) {
                currentTelemetryItem.setValue(potentiometer.getVoltage());
                myOpMode.telemetry.update();
            }

            setArmPower(0); //Whoa
        }
    }

    /**
     * moveArmUpOrDown() based on the passed targetAngle, and
     * the potentiometer's current angle.
     * @param targetAngle
     */
    public void setArmPositionUsingAngle(double targetAngle) {
        double currentAngle = getPotentiometerAngle();
        moveArmUpOrDown(currentAngle, targetAngle);
    }

    /**
     * moveArmUpOrDown() by the passed angle, relative to the current angle.
     * @param angle
     */
    public void adjustArmByAngle(double angle) {
        double currentAngle = getPotentiometerAngle();
        double targetAngle = angle + currentAngle;
        if (targetAngle > getPotentiometerAngle(getPotentiometerVoltage())) {
            targetAngle = MAX_POTENTIOMETER_ANGLE;
        }
        else if (targetAngle < 0) {
            targetAngle = 0;
        }

        moveArmUpOrDown(currentAngle, targetAngle);
    }

    /**
     * Move arm up or down until it gets to passed targetAngle.
     * @param targetAngle
     */
    private void moveArmUpOrDown(double currentAngle, double targetAngle) {
        setRunModeForAllArms(DcMotor.RunMode.RUN_USING_ENCODER);

        //Ex. If currentAngle is 100 degrees, and targetAngle is 200 degrees,
        //    we want to move arm up.
        //    If currentAngle is 200 degrees, and targetAngle is 100 degrees,
        //    we want to move arm down.
        if (currentAngle < targetAngle) {
            Telemetry.Item currentTelemetryItem = setupArmPositionTelemetry("Target Angle", targetAngle, "Current Angle", currentAngle);
            setArmPower(ARM_UP_POWER);

            while (myOpMode.opModeIsActive() && getPotentiometerAngle() < targetAngle) {
                currentTelemetryItem.setValue(getPotentiometerAngle());
                myOpMode.telemetry.update();
            }

            setArmPower(0); //Whoa
        }
        else if (currentAngle > targetAngle) {
            Telemetry.Item currentTelemetryItem = setupArmPositionTelemetry("Target Angle", targetAngle, "Current Angle", currentAngle);
            setArmPower(ARM_DOWN_POWER);

            while (myOpMode.opModeIsActive() && getPotentiometerAngle() > targetAngle) {
                currentTelemetryItem.setValue(getPotentiometerAngle());
                myOpMode.telemetry.update();
            }

            setArmPower(0); //Whoa
        }
    }
    /**
     * Set up telemetry to output this:
     *    <targetCaption> : <targetValue>
     *    <currentCaption> : <currentValue>
     * The Telemetry.Item for the currentValue is returned so the caller can keep updating it
     * using setValue().
     *
     * @param targetCaption
     * @param targetValue
     * @param currentCaption
     * @param currentValue
     * @return currentItem
     */
    private Telemetry.Item setupArmPositionTelemetry(String targetCaption, double targetValue, String currentCaption, double currentValue) {
        Telemetry telemetry = myOpMode.telemetry;
        telemetry.addData(targetCaption, targetValue);
        Telemetry.Item currentItem = telemetry.addData(currentCaption, currentValue);
        telemetry.update(); //Allow driver station to be cleared before display.
        telemetry.setAutoClear(false); //Henceforth updates should not clear display.

        return currentItem;
    }
}

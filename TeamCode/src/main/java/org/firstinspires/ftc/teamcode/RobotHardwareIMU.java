package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardwareIMU {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define all the HardwareDevices (Motors, Servos, etc.). Make them private so they can't be accessed externally.
    private IMU imu;

    // Hardware device constants.  Make them public so they can be used by the calling OpMode, if needed.
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double ROBOT_TRACK_WIDTH_INCHES = 16;
    public static final double DEFAULT_WHEEL_MOTOR_SPEED = .4;
    public static final double DEFAULT_APPROACH_SPEED = .4;

    //Update these IMU parameters for your robot.
    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    public RobotHardwareIMU(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Call init() to initialize all the robot's hardware.
     */
    public void init() {
        initIMU();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    private void initIMU() {
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        ImuOrientationOnRobot imuOrientation = new RevHubOrientationOnRobot(IMU_LOGO_DIRECTION, IMU_USB_DIRECTION);
    }

    /**
     * Return robot's current heading (yaw) in degrees.
     * @return
     */
    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Turn robot by the passed angle.
     * @param angle
     */
    public void adjustHeadingBy(double angle) {
        int leftFrontWheelPosition = leftFrontWheel.getCurrentPosition();
        int leftRearWheelPosition = leftRearWheel.getCurrentPosition();
        int rightFrontWheelPosition = rightFrontWheel.getCurrentPosition();
        int rightRearWheelPosition = rightRearWheel.getCurrentPosition();

        /**
         * wheelRotationDegrees is derived from the arc length formula in geometry,
         * where the arc length is given by the angle subtended by the arc,
         * multiplied by the radius. In this case, the radius is half of the track width.
         * Thanks ChatGPT.
         */
        double wheelRotationDegrees = (angle * ROBOT_TRACK_WIDTH_INCHES) / WHEEL_DIAMETER_INCHES;
        double countsPerDegree = COUNTS_PER_MOTOR_REV / 360;
        int wheelRotationCount = (int) (wheelRotationDegrees / countsPerDegree);

        //Increment wheel positions by wheelRotationCount.
        leftFrontWheelPosition += wheelRotationCount;
        leftRearWheelPosition += wheelRotationCount;
        rightFrontWheelPosition += wheelRotationCount;
        rightRearWheelPosition += wheelRotationCount;

        //Make right wheels go opposite the sign of the angle.
        rightFrontWheelPosition *= -1;
        rightRearWheelPosition *= -1;

        setRunModeForAllWheels(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontWheel.setTargetPosition(leftFrontWheelPosition);
        leftRearWheel.setTargetPosition(leftRearWheelPosition);
        rightFrontWheel.setTargetPosition(rightFrontWheelPosition);
        rightRearWheel.setTargetPosition(rightRearWheelPosition);
    }

    /**
     * Returns true if the passed expectedValue is within tolerance of the
     * passed actualValue. Otherwise it returns false.
     * For example, if the expectedValue is 10, the actualValue is 11, and
     * tolerance is 2, then true will be returned.
     * @param expectedValue
     * @param actualValue
     * @param tolerance
     * @return
     */
    public boolean isWithinTolerance(double expectedValue, double actualValue, double tolerance) {
        double delta = Math.abs(expectedValue - actualValue);
        return (delta <= tolerance);
    }
}

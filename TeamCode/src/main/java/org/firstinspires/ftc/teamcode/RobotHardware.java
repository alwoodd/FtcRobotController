/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * Instead of each Op Mode class redefining the robot's hardware resources within its implementation,
 * This RobotHardware class has a given robot's component resources defined and set up all in one place.
 * It also has convenience methods like driveRobot(), setArmPower(), setHandPosition(), etc. that work for that robot.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 */
public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define all the HardwareDevices (Motors, Servos, etc.). Make them private so they can't be accessed externally.
    private DcMotor leftFrontWheel;
    private DcMotor rightFrontWheel;
    private DcMotor leftRearWheel;
    private DcMotor rightRearWheel;
    private DcMotor leftArm;
    private DcMotor rightArm;
    private WebcamName webCam;
    private Servo   leftHand;
    private Servo   rightHand;
    private VisionPortal visionPortal;
    private DistanceSensor leftDistanceSensor;
    private DistanceSensor rightDistanceSensor;
    private ColorSensor colorSensor;
    private AnalogInput potentiometer;
    private IMU imu;

    // Hardware device constants.  Make them public so they can be used by the calling OpMode, if needed.
    static final double COUNTS_PER_MOTOR_REV = 560;     // Assumes 20:1 gear reduction
                                                        // See https://docs.revrobotics.com/duo-control/sensors/encoders/motor-based-encoders
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double WHEEL_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double ENCODER_COUNT_PER_DEGREE = COUNTS_PER_MOTOR_REV / 360;
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double WHEEL_DRIVE_GAIN_FACTOR = .03;  // Larger is more responsive, but also less stable
    static final double WHEEL_TURN_GAIN_FACTOR  = .02;  // Larger is more responsive, but also less stable

    static final double ROBOT_TRACK_WIDTH_INCHES = 16;
    static final double DEFAULT_WHEEL_MOTOR_SPEED = .4;
    static final double  HEADING_THRESHOLD = 1.0 ; // How close must the heading get to the target before moving to next step.
                                                   // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    static final double MID_SERVO       =  0.5 ;
    static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    static final double ARM_UP_POWER    =  0.45 ;
    static final double ARM_DOWN_POWER  = -0.45 ;
    static final double MAX_POTENTIOMETER_ANGLE = 270;
    static final double DEFAULT_APPROACH_SPEED = .4;

    static final double SENSOR_DISTANCE_OUT_OF_RANGE = 20;

    //Update these IMU parameters for your robot.
    static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    /**
     * You can set the arm positions using angles and/or potentiometer voltage.
     * Tune these values for your robot's actual values.
     */
    static final double ARM_PARKED_ANGLE = 0;
    static final double ARM_PIXEL_PICKUP_ANGLE = 200;
    static final double ARM_BACKDROP_ANGLE = 100;
    static final double ARM_PARKED_VOLTAGE = 0;
    static final double ARM_PIXEL_PICKUP_VOLTAGE = 3;
    static final double ARM_BACKDROP_VOLTAGE = 1.4;

    /**
     * The one and only constructor requires a reference to an OpMode.
     * @param opmode
     */
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Call init() to initialize all the robot's hardware.
     */
    public void init() {
        initWheelMotors();
        initServos();
        initDistanceSensors();
        initColorSensor();
        initAnalogInputs();
        initArmMotors();
        initIMU();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Call shutDown() to stop and close all the robot's hardware.
     */
/*
    public void shutDown() {
        tfod.shutdown();
        visionPortal.close();
    }
*/
    /**
     * Initialize all the wheel motors.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    private void initWheelMotors()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontWheel  = myOpMode.hardwareMap.get(DcMotor.class, "LFront");
        rightFrontWheel = myOpMode.hardwareMap.get(DcMotor.class, "RFront");
        leftRearWheel = myOpMode.hardwareMap.get(DcMotor.class, "LRear");
        rightRearWheel = myOpMode.hardwareMap.get(DcMotor.class, "RRear");

        // To drive forward, most robots need the motors on one side to be reversed, because the axles point in opposite directions.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        leftRearWheel.setDirection(DcMotor.Direction.FORWARD);
        rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        rightRearWheel.setDirection(DcMotor.Direction.REVERSE);

        // Set wheel motors to not resist turning when motor is stopped.
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Use STOP_AND_RESET_ENCODER to keep wheels from moving in unexpected ways
        // during subsequent runs of an OpMode.
        setRunModeForAllWheels(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Initialize all servos.
     */
    private void initServos() {
        // Define and initialize ALL installed servos.
        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);
    }

    /**
     * Initialize distance sensor(s).
     */
    private void initDistanceSensors() {
        leftDistanceSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "centerDistanceSensor");
        rightDistanceSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "sideDistanceSensor");
    }

    private void initColorSensor() {
        colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    private void initAnalogInputs() {
        potentiometer = myOpMode.hardwareMap.get(AnalogInput.class, "potentiometer");
    }

    private void initArmMotors() {
        leftArm = myOpMode.hardwareMap.get(DcMotor.class, "leftArm");
        rightArm = myOpMode.hardwareMap.get(DcMotor.class, "rightArm");
    }

    private void initIMU() {
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        //Define hub orientation.
        ImuOrientationOnRobot imuOrientation = new RevHubOrientationOnRobot(IMU_LOGO_DIRECTION, IMU_USB_DIRECTION);
        //Initialize IMU instance with hub orientation.
        imu.initialize((new IMU.Parameters(imuOrientation)));
        //Reset heading/yaw to zero.
        imu.resetYaw();
    }

    /**
     * Drive robot to the targeted position designated by the passed leftInches and
     * rightInches, at the power specified by speed.
     * @param leftInches How far the left wheels should travel
     * @param rightInches How far the right wheels should travel
     * @param speed speed applied to all wheels
     */
    public void autoDriveRobot(int leftInches, int rightInches, double speed) {
        /**
         * Some robots, when running back-to-back calls to autoDriveRobot(),
         * start driving some of their wheels in unexpected directions. Uncomment
         * this if your robot starts acting stupid:
         *
         * setRunModeForAllWheels(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         */

        int leftInchesToCPI = (int) (leftInches * WHEEL_COUNTS_PER_INCH);
        int rightInchesToCPI = (int) (rightInches * WHEEL_COUNTS_PER_INCH);

        int leftFrontTarget = leftFrontWheel.getCurrentPosition() + leftInchesToCPI;
        int leftRearTarget = leftRearWheel.getCurrentPosition() + leftInchesToCPI;
        int rightFrontTarget = rightFrontWheel.getCurrentPosition() + rightInchesToCPI;
        int rightRearTarget = rightRearWheel.getCurrentPosition() + rightInchesToCPI;

        leftFrontWheel.setTargetPosition(leftFrontTarget);
        leftRearWheel.setTargetPosition(leftRearTarget);
        rightFrontWheel.setTargetPosition(rightFrontTarget);
        rightRearWheel.setTargetPosition(rightRearTarget);

        setRunModeForAllWheels(DcMotor.RunMode.RUN_TO_POSITION);

        //Set up telemetry
        myOpMode.telemetry.setAutoClear(false);
        myOpMode.telemetry.addData("Heading", "Current Wheel Positions");
        Telemetry.Item leftFrontWheelItem = myOpMode.telemetry.addData("LF Wheel", leftFrontWheel.getCurrentPosition());
        Telemetry.Item leftRearWheelItem = myOpMode.telemetry.addData("LR Wheel", leftRearWheel.getCurrentPosition());
        Telemetry.Item rightFrontWheelItem = myOpMode.telemetry.addData("RF Wheel", rightFrontWheel.getCurrentPosition());
        Telemetry.Item rightRearWheelItem = myOpMode.telemetry.addData("RR Wheel", rightRearWheel.getCurrentPosition());
        myOpMode.telemetry.update();

        //It is best for power to be positive for RUN_TO_POSITION.
        setPowerAllWheels(Math.abs(speed));

        // Update telemetry for as long as the wheel motors isBusy().
        while (leftFrontWheel.isBusy() && leftRearWheel.isBusy() && rightFrontWheel.isBusy() && rightRearWheel.isBusy()) {
            leftFrontWheelItem.setValue(leftFrontWheel.getCurrentPosition());
            leftRearWheelItem.setValue(leftRearWheel.getCurrentPosition());
            rightFrontWheelItem.setValue(rightFrontWheel.getCurrentPosition());
            rightRearWheelItem.setValue(rightRearWheel.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        //Robot has RUN_TO_POSITION.
        setPowerAllWheels(0); //Whoa.
        myOpMode.telemetry.setAutoClear(true);
    }

    /**
     * autoDriveRobot using DEFAULT_WHEEL_MOTOR_SPEED.
     * @param leftInches How far the left wheels should travel
     * @param rightInches How far the right wheels should travel
     */
    public void autoDriveRobot(int leftInches, int rightInches) {
        autoDriveRobot(leftInches, rightInches, DEFAULT_WHEEL_MOTOR_SPEED);
    }

    /**
     * Drive robot the targeted inches, at the passed heading, at the passed speed.
     * Left and right wheel speeds are adjusted as necessary to get the robot on
     * the passed heading.
     * @param inches How far the robot should travel
     * @param heading Heading in yaw degree (+/- 180)
     * @param speed Speed to apply to all wheels
     */
    public void autoDriveRobotWithHeading(int inches, double heading, double speed) {
        int inchesToCPI = (int) (inches * WHEEL_COUNTS_PER_INCH);

        int leftFrontTarget = leftFrontWheel.getCurrentPosition() + inchesToCPI;
        int leftRearTarget = leftRearWheel.getCurrentPosition() + inchesToCPI;
        int rightFrontTarget = rightFrontWheel.getCurrentPosition() + inchesToCPI;
        int rightRearTarget = rightRearWheel.getCurrentPosition() + inchesToCPI;

        leftFrontWheel.setTargetPosition(leftFrontTarget);
        leftRearWheel.setTargetPosition(leftRearTarget);
        rightFrontWheel.setTargetPosition(rightFrontTarget);
        rightRearWheel.setTargetPosition(rightRearTarget);

        setRunModeForAllWheels(DcMotor.RunMode.RUN_TO_POSITION);

        HeadingTelemetry headingTelemetry = new HeadingTelemetry("Drive");
        headingTelemetry.setTargetPositions(leftFrontTarget, leftRearTarget, rightFrontTarget, rightRearTarget);
        headingTelemetry.setTargetHeading(heading);

        // Set the required driving speed. Use a positive amount here.
        // Start driving straight, and then enter the control loop
        setPowerAllWheels(Math.abs(speed));

        while (myOpMode.opModeIsActive() && leftFrontWheel.isBusy() && leftRearWheel.isBusy() && rightFrontWheel.isBusy() && rightRearWheel.isBusy()) {
            double headingSpeedAdjustment = getHeadingCorrection(heading, WHEEL_DRIVE_GAIN_FACTOR, speed);

            // if driving in reverse, the motor correction also needs to be reversed
            if (inches < 0)
                headingSpeedAdjustment *= -1.0;

            headingTelemetry.updateHeadingData(headingSpeedAdjustment);
            setPowerAllWheels(speed, headingSpeedAdjustment);
            headingTelemetry.updateWheelData();
        }

        setPowerAllWheels(0); //Whoa
        headingTelemetry.reset();
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the OpMode running.
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                If a relative angle is required, add/subtract from current heading.
     * @param speed Desired speed of turn. (range 0 to +speed)
     */
    public void turnToHeading(double heading, double speed) {
        setRunModeForAllWheels(DcMotor.RunMode.RUN_USING_ENCODER);

        HeadingTelemetry headingTelemetry = new HeadingTelemetry("Turning");
        headingTelemetry.setTargetHeading(heading);

        // Initialize headingDelta with diff between desired and actual heading,
        // then recalculate during while loop.
        double headingDelta = heading - getHeadingDegrees();
        double headingSpeedAdjustment;

        // keep looping while we are still active, and not on heading.
        while (myOpMode.opModeIsActive() && (Math.abs(headingDelta) > HEADING_THRESHOLD)) {
            // Determine required steering to keep on heading
            headingSpeedAdjustment = getHeadingCorrection(heading, WHEEL_TURN_GAIN_FACTOR, speed);
            headingTelemetry.updateHeadingData(headingSpeedAdjustment);

            // Pivot in place by applying the turning correction
            setPowerAllWheels(0, headingSpeedAdjustment);
            headingTelemetry.updateWheelData();

            headingDelta = heading - getHeadingDegrees();
        }

        setPowerAllWheels(0); //Whoa
        headingTelemetry.reset();
    }

    /**
     * Calculate a motor speed that can be used to turn the robot to the desiredHeading.
     * Get the difference between the passed desiredHeading and the actual heading.
     * Normalize the difference to be in terms of yaw (+/- 180 degrees).
     * Multiply the difference by gainFactor to get a potential motor speed.
     * Clip the potential motor speed to the maxSpeed.
     * @param desiredHeading yaw degrees (+/- 180)
     * @param gainFactor factor to convert heading difference to a motor speed
     * @param maxSpeed maximum speed returned.
     * @return motor speed in the range of -maxSpeed to maxSpeed.
     */
    public double getHeadingCorrection(double desiredHeading, double gainFactor, double maxSpeed) {
        double headingError = desiredHeading - this.getHeadingDegrees();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction.
        // Clip the correction to the maximum permitted value.
        return Range.clip(headingError * gainFactor, -maxSpeed, maxSpeed);
    }

    /**
     * Set the RunMode for all wheel motors to the passed runMode.
     * @param runMode DcMotor.RunMode
     */
    public void setRunModeForAllWheels(DcMotor.RunMode runMode) {
        leftFrontWheel.setMode(runMode);
        leftRearWheel.setMode(runMode);
        rightFrontWheel.setMode(runMode);
        rightRearWheel.setMode(runMode);
    }

    /**
     * Adjust the baseSpeed by  +/- headingSpeedAdjustment, then set power to the left
     * and right wheels to those powers. Scale down both left and right motor speeds
     * if either is > 1.0 or < -1.0.
     * @param baseSpeed - base speed of the motor to which headingSpeedAdjustment will be applied.
     * @param headingSpeedAdjustment - power adjustment applied to baseSpeed to turn the robot to the desiredHeading.
     */
    public void setPowerAllWheels(double baseSpeed, double headingSpeedAdjustment) {
        /*
        *
         * If your robot uses a negative power to move forward,
         * you'll need to subtract for leftSpeed, and add for
         * rightSpeed.
         */
        double leftSpeed = baseSpeed + headingSpeedAdjustment;
        double rightSpeed = baseSpeed - headingSpeedAdjustment;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftFrontWheel.setPower(leftSpeed);
        leftRearWheel.setPower(leftSpeed);
        rightFrontWheel.setPower(rightSpeed);
        rightRearWheel.setPower(rightSpeed);
    }

    /**
     * Set the Power for all wheels to the passed speed.
     * @param speed - power of the motor, a value in the interval [-1.0, 1.0]
     */
    public void setPowerAllWheels(double speed) {
        leftFrontWheel.setPower(speed);
        leftRearWheel.setPower(speed);
        rightFrontWheel.setPower(speed);
        rightRearWheel.setPower(speed);
    }

    /**
     * Drive robot according to passed stick inputs.
     * @param stick1X Value from stick 1's X axis
     * @param stick1Y Value from stick 1's Y axis
     * @param stick2X Value from stick 2's X axis
     */
    public void manuallyDriveRobot(double stick1X, double stick1Y, double stick2X) {
        double vectorLength = Math.hypot(stick1X, stick1Y);
        double robotAngle = Math.atan2(stick1Y, -stick1X) - Math.PI / 4;
        double rightXscale = stick2X * .5;
        double rightFrontVelocity = vectorLength * Math.cos(robotAngle) + rightXscale;
        double leftFrontVelocity = vectorLength * Math.sin(robotAngle) - rightXscale;
        double rightRearVelocity = vectorLength * Math.sin(robotAngle) + rightXscale;
        final double leftRearVelocity = vectorLength * Math.cos(robotAngle) - rightXscale;

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy.
        setRunModeForAllWheels(DcMotor.RunMode.RUN_USING_ENCODER);

        // Use existing method to drive both wheels.
        setDrivePower(leftFrontVelocity, rightFrontVelocity, leftRearVelocity, rightRearVelocity);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        // Output the values to the motor drives.
        leftFrontWheel.setPower(leftFrontPower);
        rightFrontWheel.setPower(rightFrontPower);
        leftRearWheel.setPower(leftRearPower);
        rightRearWheel.setPower(rightRearPower);
    }

    /**
     * Return robot's current heading (yaw) in degrees.
     * @return heading in degrees
     */
    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Calculate the delta between expectedHeading and actualHeading.
     * @param expectedHeading
     * @param actualHeading
     * @return headingDelta
     */
/*
    public double headingDeltaDegrees(double expectedHeading, double actualHeading) {
        // Ensure both angles are in the range [0, 360]
        expectedHeading = normalizeAngle(expectedHeading);
        actualHeading = normalizeAngle(actualHeading);

        // Calculate the heading error considering wrap-around
        double headingDelta = expectedHeading - actualHeading;

        // Adjust for wrap-around (ensure the error is in the range [-180, 180))
        if (headingDelta > 180.0) {
            headingDelta -= 360.0;
        } else if (headingDelta <= -180.0) {
            headingDelta += 360.0;
        }

        return headingDelta;
    }
*/

    /**
     * Calculate the delta between the expectedHeading and that returned by getHeadingDegrees().
     * @param expectedHeading
     * @return headingDelta
     */
/*
    public double headDeltaDegrees(double expectedHeading) {
        return headingDeltaDegrees(expectedHeading, getHeadingDegrees());
    }
*/

    /**
     * Turn robot by the passed angle.
     * @param angle
     */
    @Deprecated
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
    public static boolean isWithinTolerance(double expectedValue, double actualValue, double tolerance) {
        double delta = Math.abs(expectedValue - actualValue);
        return (delta <= tolerance);
    }

    /**
     * Given any angle, returns value that will be 0 - 360.
     * For example, -400 is normalized to 320.
     * @param angle
     * @return normalized angle
     */
/*
    private double normalizeAngle(double angle) {
        // Normalize the angle to be in the range [0, 360]
        return (angle % 360.0 + 360.0) % 360.0;
    }
*/

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
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);
    }

    /**
     * Move XYZ servo so that drone is released.
     */
    public void releaseDrone() {
        //Move whichever servo or other HardwareDevice.
    }

    /**
     * Return distance detected by Left Sensor in CM.
     * @return distance in CM
     */
    public double getLeftSensorDistanceInCM() {
        return leftDistanceSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Return distance detected by Left Sensor,
     * using whatever DistanceUnit is passed in.
     * Available units:
     *    DistanceUnit.MM
     *    DistanceUnit.CM
     *    DistanceUnit.METER
     *    DistanceUnit.INCH
     * @param distanceUnit
     * @return
     */
    public double getLeftSensorDistance(DistanceUnit distanceUnit) {
        return leftDistanceSensor.getDistance(distanceUnit);
    }

    /**
     * Return distance detected by Right Sensor in CM.
     * @return distance in CM
     */
    public double getRightSensorDistanceInCM() {
        return rightDistanceSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Return distance detected by Right Sensor,
     * using whatever DistanceUnit is passed in.
     * Available units:
     *    DistanceUnit.MM
     *    DistanceUnit.CM
     *    DistanceUnit.METER
     *    DistanceUnit.INCH
     * @param distanceUnit
     * @return
     */
    public double getRightSensorDistance(DistanceUnit distanceUnit) {
        return rightDistanceSensor.getDistance(distanceUnit);
    }

    /**
     * Return all color values from Color Sensor.
     * @return RGBAcolors
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
     * In this example, if neither the left or right sensors detect an object,
     * the position is 2. If the left sensor detects an object, the position is 1. Lastly,
     * if the right sensor detects an object, the position is 3.
     *
     * @return int positionNumber
     */
    public int getSpikeObjectPosition() {
        double leftSensorDistance = getLeftSensorDistanceInCM();
        double rightSensorDistance = getRightSensorDistanceInCM();
        int positionNumber = 0;

        if (leftSensorDistance >= SENSOR_DISTANCE_OUT_OF_RANGE && rightSensorDistance >= SENSOR_DISTANCE_OUT_OF_RANGE) {
            myOpMode.telemetry.addData("NOT DETECTED", "Object not detected by any sensor!");
            positionNumber = 2;
        }
        else if (leftSensorDistance <= SENSOR_DISTANCE_OUT_OF_RANGE) {
            myOpMode.telemetry.addData("DETECTED LEFT", "Object distance is %.0f CM", leftSensorDistance);
            positionNumber = 1;
        }
        else {
            myOpMode.telemetry.addData("DETECTED RIGHT", "Object distance is %.0f CM", rightSensorDistance);
            positionNumber = 3;
        }
        myOpMode.telemetry.update();

        return positionNumber;
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
        // If targetAngle > the angle using potentiometer's max voltage,
        // set targetAngle to MAX_POTENTIOMETER_ANGLE.
        if (targetAngle > getPotentiometerAngle(potentiometer.getMaxVoltage())) {
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
     * Adjust arm angles to the passed angle using encoders.
     * @param angle
     */
    public void adjustArmAngleUsingEncoder(double angle) {
        int encoderCountAdjustment = (int) (ENCODER_COUNT_PER_DEGREE * angle);

        //The arms *should* be parallel, but calculate new targets for both arms anyway.
        int leftArmTargetEncoderCount = leftArm.getCurrentPosition() + encoderCountAdjustment;
        int rightArmTargetEncoderCount = rightArm.getCurrentPosition() + encoderCountAdjustment;

        leftArm.setTargetPosition(leftArmTargetEncoderCount);
        rightArm.setTargetPosition(rightArmTargetEncoderCount);
        setRunModeForAllArms(DcMotor.RunMode.RUN_TO_POSITION);

        //Set up telemetry
        myOpMode.telemetry.setAutoClear(false);
        Telemetry.Item leftArmItem = myOpMode.telemetry.addData("Left Arm", leftArm.getCurrentPosition());
        Telemetry.Item rightArmItem = myOpMode.telemetry.addData("Right Arm", rightArm.getCurrentPosition());
        myOpMode.telemetry.update();

        setArmPower(ARM_DOWN_POWER); //Using ARM_DOWN_POWER for now.

        // Update telemetry for as long as the wheel motors isBusy().
        while (leftArm.isBusy() && rightArm.isBusy()) {
            leftArmItem.setValue(leftArm.getCurrentPosition());
            rightArmItem.setValue(rightArm.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        myOpMode.telemetry.setAutoClear(true);
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

    /**
     * Provides a convenient interface for setting and updating telemetry for
     * robot movements during heading-driven events.
     */
    public class HeadingTelemetry {
        private Telemetry.Item leftTargetPositionsItem;
        private Telemetry.Item rightTargetPositionsItem;
        private Telemetry.Item leftActualPositionsItem;
        private Telemetry.Item rightActualPositionsItem;
        private Telemetry.Item headingItem;
        private Telemetry.Item headingAdjustmentItem;
        private Telemetry.Item leftWheelSpeedsItem;
        private Telemetry.Item rightWheelSpeedsItem;
        private Telemetry telemetry = myOpMode.telemetry;
        private double targetHeading = 0;
        private double currentHeading = 0;

        /**
         * Construct and set up all telemetry.
         * Passing anything but "Turning" (case insensitive) causes wheel targets and positions
         * to also be sent to telemetry.
         * @param motionHeading value such as "Driving" or "Turning"
         */
        public HeadingTelemetry(String motionHeading) {
            telemetry.setAutoClear(false);
            telemetry.addData("Motion", motionHeading);

            if (!"Turning".equalsIgnoreCase(motionHeading)) {
                telemetry.addData("Wheel Targets", "");
                leftTargetPositionsItem = telemetry.addData("Left Tgt F:R", "%7d:%7d", 0, 0);
                rightTargetPositionsItem = telemetry.addData("Right Tgt F:R", "%7d:%7d", 0, 0);

                telemetry.addData("Wheel Positions", "");
                leftActualPositionsItem = telemetry.addData("Left Pos F:R", "%7d:%7d", 0, 0);
                rightActualPositionsItem = telemetry.addData("Right Pos F:R", "%7d:%7d", 0, 0);
            }

            headingItem = telemetry.addData("Heading-Target:Current", "%5.2f : %5.0f", targetHeading, currentHeading);
            headingAdjustmentItem = telemetry.addData("Heading Adj:Speed Adj", "%5.1f : %5.1f",0.0, 0.0);

            telemetry.addData("Wheel Speeds", "");
            leftWheelSpeedsItem = telemetry.addData("Left Whl Spds F:R", "%5.2f : %5.2f", 0.0, 0.0);
            rightWheelSpeedsItem = telemetry.addData("Right Whl Spds F:R","%5.2f : %5.2f", 0.0, 0.0);
        }

        /**
         * Set target wheel positions.
         * If constructed with "Turning" (case insensitive), this method does nothing.
         * @param leftFrontPosition
         * @param leftRearPosition
         * @param rightFrontPosition
         * @param rightRearPosition
         */
        public void setTargetPositions(int leftFrontPosition, int leftRearPosition, int rightFrontPosition, int rightRearPosition) {
            if (leftTargetPositionsItem == null)
                return;
            leftTargetPositionsItem.setValue("%7d:%7d", leftFrontPosition, leftRearPosition);
            rightTargetPositionsItem.setValue("%7d:%7d", rightFrontPosition, rightRearPosition);
            telemetry.update();
        }

        /**
         * Update actual wheel positions and speeds.
         * If constructed with "Turning" (case insensitive), actual position data is ignored.
         */
        public void updateWheelData() {
            if (leftActualPositionsItem != null) {
                leftActualPositionsItem.setValue("%7d:%7d", leftFrontWheel.getCurrentPosition(), leftRearWheel.getCurrentPosition());
                rightActualPositionsItem.setValue("%7d:%7d", rightFrontWheel.getCurrentPosition(), rightRearWheel.getCurrentPosition());
            }

            leftWheelSpeedsItem.setValue("%5.2f : %5.2f", leftFrontWheel.getPower(), leftRearWheel.getPower());
            rightWheelSpeedsItem.setValue("%5.2f : %5.2f", rightFrontWheel.getPower(), rightRearWheel.getPower());

            telemetry.update();
        }

        /**
         * Set the target heading telemetry
         * @param heading Target
         */
        public void setTargetHeading(double heading) {
            targetHeading = heading;
            headingItem.setValue("%5.2f : %5.0f", targetHeading, currentHeading);
            telemetry.update();
        }

        /**
         * Update all heading telemetry.
         * @param speedAdjustment Speed adjustment to show in telemetry
         */
        public void updateHeadingData(double speedAdjustment) {
            currentHeading = getHeadingDegrees();
            headingItem.setValue("%5.2f : %5.0f", targetHeading, currentHeading);
            headingAdjustmentItem.setValue("%5.1f : %5.1f", targetHeading - currentHeading, speedAdjustment);
            telemetry.update();
        }

        /**
         * Telemetry Auto Clear is set to false when this class is constructed.
         * Call this once when finished with the instance to set Auto Clear to true.
         */
        public void reset() {
            this.telemetry.setAutoClear(true);
        }
    }
}
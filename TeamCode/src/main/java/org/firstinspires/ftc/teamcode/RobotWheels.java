package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class contains only  team’s wheel motor component resources. Use this class if your team
 * is not using Pedro Pathing.
 * Additionally, this class has convenience methods, such as autoDriveRobot() and manuallyDriveRobot().
 * Each OpMode then uses an instance of the single RobotWheels (if not using Pedro Pathing).
 */
public class RobotWheels {
    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
    private final RobotHardware robotHardware;

    // Define all the HardwareDevices (Motors, Servos, etc.). Make them private so they can't be accessed externally.
    private DcMotor leftFrontWheel;
    private DcMotor rightFrontWheel;
    private DcMotor leftRearWheel;
    private DcMotor rightRearWheel;

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
   static final double DEFAULT_APPROACH_SPEED = .4;


    /**
     * The one and only constructor requires a reference to a LinearOpMode, and the RobotHardware.
     * @param opmode
     */
    public RobotWheels(LinearOpMode opmode, RobotHardware robotHardware) {
        this.myOpMode = opmode;
        this.robotHardware = robotHardware;
    }

    /**
     * Call init() to initialize all the robot's hardware.
     */
    public void init() {
        initWheelMotors();

    }
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
        double headingDelta = heading - robotHardware.getHeadingDegrees();
        double headingSpeedAdjustment;

        // keep looping while we are still active, and not on heading.
        while (myOpMode.opModeIsActive() && (Math.abs(headingDelta) > HEADING_THRESHOLD)) {
            // Determine required steering to keep on heading
            headingSpeedAdjustment = getHeadingCorrection(heading, WHEEL_TURN_GAIN_FACTOR, speed);
            headingTelemetry.updateHeadingData(headingSpeedAdjustment);

            // Pivot in place by applying the turning correction
            setPowerAllWheels(0, headingSpeedAdjustment);
            headingTelemetry.updateWheelData();

            headingDelta = heading - robotHardware.getHeadingDegrees();
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
        double headingError = desiredHeading - robotHardware.getHeadingDegrees();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction.
        // Clip the correction to the maximum permitted value.
        return Range.clip(headingError * gainFactor, -maxSpeed, maxSpeed);
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
            colors = robotHardware.getSensorColors();
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
            currentHeading = robotHardware.getHeadingDegrees();
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
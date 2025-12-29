package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp
public class SimpleSwyftTest extends LinearOpMode {
    private DcMotor leftFrontWheel;
    private DcMotor rightFrontWheel;
    private DcMotor leftRearWheel;
    private DcMotor rightRearWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        initWheelMotors();

        waitForStart();
        setRunModeForAllWheels(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean isFwd = true;

        while (opModeIsActive()) {
            if (gamepad1.bWasPressed()) {
                isFwd = !isFwd;
            }

            double speed = gamepad1.right_trigger;
            if (!isFwd) {
                speed *= -1;
            }
            telemetry.addData("right trigger", speed);
            telemetry.addData("Forward?", isFwd);
            telemetry.update();
            setPowerAllWheels(speed);
        }
    }

    private void initWheelMotors()    {
        // Define and Initialize Motors.
        leftFrontWheel  = hardwareMap.get(DcMotor.class, "motor_lf");
        leftRearWheel = hardwareMap.get(DcMotor.class, "motor_lb");
        rightFrontWheel = hardwareMap.get(DcMotor.class, "motor_rf");
        rightRearWheel = hardwareMap.get(DcMotor.class, "motor_rb");

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

        MotorConfigurationType mct = leftFrontWheel.getMotorType();
        mct.setTicksPerRev(537.7);
        leftFrontWheel.setMotorType(mct);
        leftRearWheel.setMotorType(mct);
        rightFrontWheel.setMotorType(mct);
        rightRearWheel.setMotorType(mct);
    }

    private void setPowerAllWheels(double speed) {
        leftFrontWheel.setPower(speed);
        leftRearWheel.setPower(speed);
        rightFrontWheel.setPower(speed);
        rightRearWheel.setPower(speed);
    }

    private void setRunModeForAllWheels(DcMotor.RunMode runMode) {
        leftFrontWheel.setMode(runMode);
        leftRearWheel.setMode(runMode);
        rightFrontWheel.setMode(runMode);
        rightRearWheel.setMode(runMode);
    }
}

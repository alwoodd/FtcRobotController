package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Use arm")
public class CenterstageUseArm extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        //Assume robot is already positioned.
        while (opModeIsActive()) {
            //Move arm to "parked" position.
            //Set arm positions using either angles or voltages.
            if (gamepad1.a) {
                robot.setArmPositionUsingAngle(RobotHardware.ARM_PARKED_ANGLE);
                //robot.setArmPositionUsingVoltage(RobotHardware.ARM_PARKED_VOLTAGE);
            }
            //Move arm to pixel pickup position.
            else if (gamepad1.b) {
                robot.setArmPositionUsingAngle(RobotHardware.ARM_PIXEL_PICKUP_ANGLE);
                //robot.setArmPositionUsingVoltage(RobotHardware.ARM_PIXEL_PICKUP_VOLTAGE);
            }
            //Move arm to position to place pixel on backdrop.
            else if (gamepad1.x) {
                robot.setArmPositionUsingAngle(RobotHardware.ARM_BACKDROP_ANGLE);
                //robot.setArmPositionUsingVoltage(RobotHardware.ARM_BACKDROP_VOLTAGE);
            }
        }
    }
}

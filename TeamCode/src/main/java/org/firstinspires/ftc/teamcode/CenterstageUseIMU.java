package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class CenterstageUseIMU extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        while (opModeIsActive()) {
            //Drive straight
            robot.autoDriveRobot(50, 50);
            //Drive 50 inches on a -90 degree heading.
            robot.autoDriveRobotWithHeading(50, -90, RobotHardware.DEFAULT_WHEEL_MOTOR_SPEED);
            //Turn left 45 degrees.
            robot.turnToHeading(45, RobotHardware.DEFAULT_WHEEL_MOTOR_SPEED);
        }
    }
}

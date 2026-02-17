package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class CenterstageUseIMU extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        RobotWheels robotWheels = new RobotWheels(this, robot);

        while (opModeIsActive()) {
            //Drive straight
            robotWheels.autoDriveRobot(50, 50);
            //Drive 50 inches on a -90 degree heading.
            robotWheels.autoDriveRobotWithHeading(50, -90, RobotHardware.DEFAULT_MOTOR_SPEED);
            //Turn left 45 degrees.
            robotWheels.turnToHeading(45, RobotHardware.DEFAULT_MOTOR_SPEED);
        }
    }
}

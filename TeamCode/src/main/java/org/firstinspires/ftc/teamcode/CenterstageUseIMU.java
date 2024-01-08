package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class CenterstageUseIMU extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        double expectedHeading = 90;
        double actualHeading;
        double tolerance = 4; //degrees

        while (opModeIsActive()) {
            robot.autoDriveRobot(50, 50);  //Drive straight
            robot.autoDriveRobot(50, -50); //Turn 90 degrees
            actualHeading = robot.getHeadingDegrees();
            if (!robot.isWithinTolerance(expectedHeading, actualHeading, tolerance)) {
                robot.adjustHeadingBy(expectedHeading - actualHeading);
            }
        }
    }
}

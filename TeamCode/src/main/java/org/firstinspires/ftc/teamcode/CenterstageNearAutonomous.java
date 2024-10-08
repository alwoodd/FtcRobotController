package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This 2023-2024 OpMode shows a way to simply park the robot in the alliance's backstage area.
 */
@Autonomous(name = "Park starting near backstage")
public class CenterstageNearAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        /**
         * Park the robot by using autoDriveRobot to drive it straight.
         * Do this by setting both left and right sides of the robot to the same amount.
         */
        while (opModeIsActive()) {
            //Your robot may need different values. YMMV.
            int leftInches = 50;
            int rightInches = 50;
            robot.autoDriveRobot(leftInches, rightInches);
        }
    }
}

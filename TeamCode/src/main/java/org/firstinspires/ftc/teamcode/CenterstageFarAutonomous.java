package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This 2023-2024 OpMode shows a way to simply park the robot in the alliance's backstage area.
 */
@Autonomous(name = "Park from red front stage") //Or "Park from blue front stage"
public class CenterstageFarAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        /**
         * Park the robot by using autoDriveRobot to drive it straight, turn it, then drive
         * straight again.
         */
        while (opModeIsActive()) {
            //Your robot may need different values. YMMV.
            robot.autoDriveRobot(50, 50);  //Straight
            robot.autoDriveRobot(20, -20); //Turn
            robot.autoDriveRobot(50, 50);  //Straight again
        }
    }
}

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotWheels;

@Autonomous(name = "Blue Autonomous Primary")
public class BlueAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        RobotWheels robotWheels = new RobotWheels(this, robot);
        robot.init();
        robotWheels.init();

        waitForStart();

        //Drive straight 50 inches.
        robotWheels.autoDriveRobot(50, 50);

        while (opModeIsActive()) {
            robot.startIntake();
            //If intake sees the right color, break while loop.
            if (robot.isRightColor(ObjectColor.BLUE)) {
                break;
            }
            //Else spit out the object and continue while loop.
            else {
                robot.runOuttake(1000);
            }
        }

        //We have an object of the desired color.
        robot.stopIntake();
    }
}
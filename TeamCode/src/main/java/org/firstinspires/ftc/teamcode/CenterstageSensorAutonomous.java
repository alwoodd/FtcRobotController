package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DistanceSensor.distanceOutOfRange;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This 2023-2024 OpMode shows a way to detect an image,
 * determine the position number it is in, then move
 * the robot to it. It also demonstrates:
 *  -Dropping a pixel on the correct spike mark
 *  -Driving to the backdrop in front of the left, middle, or right side.
 *  -Placing the pixel onto the backdrop.
 */
@Autonomous(name = "Detect object and move robot to it")
public class CenterstageSensorAutonomous extends LinearOpMode {
    private  RobotHardware robot;
    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);
        robot.init();

        int colorThreshold = 500; //Tune for expected color (blue or red).

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {
            robot.driveToSpike(SpikeColor.RED, colorThreshold);
            int positionNumber = robot.getSpikeObjectPosition();
            //Turning robot to spike position also moves pixel onto correct spike.
            turnToSpike(positionNumber);
            driveToBackdrop(positionNumber);
            placePixelOnBackdrop(); //Potentially refactor to separate class
        }

    }   // end runOpMode()

    /**
     * Move robot to spike mark corresponding to passed positionNumber.
     * @param positionNumber
     */
    private void turnToSpike(int positionNumber) {
        int driveLeftInches = 0;
        int driveRightInches = 0;

        /**
         * If object position is:
         *   1: Turn robot to to left spike
         *   2: Do NOT turn robot. Is is already facing middle spike.
         *   3: Turn robot to right spike
         */
        switch (positionNumber) {
            case 1:
                driveLeftInches = -22;
                driveRightInches = -22;
                break;
            case 2:
                driveLeftInches = 0;
                driveRightInches = 0;
                break;
            case 3:
                driveLeftInches = -24;
                driveRightInches = -24;
                break;
        }

        robot.autoDriveRobot(driveLeftInches, driveRightInches);
    }

    /**
     * Drive from current position to backdrop,
     * then position robot in front of left, middle right backstage area
     * corresponding to passed positionNumber.
     * @param positionNumber
     */
    private void driveToBackdrop(int positionNumber) {
    }

    /**
     * Place pixel onto backdrop.
     * It is assumed the robot is already correctly positioned.
     */
    private void placePixelOnBackdrop() {
    }
}   // end class

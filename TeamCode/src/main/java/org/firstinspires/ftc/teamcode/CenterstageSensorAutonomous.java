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
@Autonomous(name = "Detect image and move robot to it")
public class CenterstageSensorAutonomous extends LinearOpMode {
    private RobotHardware robot;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);
        robot.init();

        int positionNumber;

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {
            driveToSpike(SpikeColor.RED);
            positionNumber = getObjectPosition();
            //Turning robot to spike position also moves pixel onto correct spike.
            turnToSpike(positionNumber);
            driveToBackdrop(positionNumber);
            placePixelOnBackdrop(); //Potentially refactor to separate class
        }

        robot.shutDown();
    }   // end runOpMode()

    /**
     * Drive robot until the passed color is detected.
     * @param color
     */
    private void driveToSpike(SpikeColor color) {
        double approachSpeed = .25;
        int colorThreshold = 500; //Tune for expected color (blue or red).
        RGBAcolors colors;

        robot.setPowerAllWheels(approachSpeed);

        while (opModeIsActive()) {
            colors = robot.getSensorColors();
            if (color == SpikeColor.RED && colors.getRed() > colorThreshold) {
                break;
            }
            else if (colors.getBlue() > colorThreshold) {
                break;
            }
        }

        robot.setPowerAllWheels(0);
    }

    /**
     * Determine which position (1, 2, or 3) the sensor detects an object (such as a cube) in.
     * If the sensors do not detect an object using either the front or side sensors,
     * it is assumed the object is in the 3rd position.
     * @return int positionNumber
     */
    private int getObjectPosition() {
        double centerSensorDistance = robot.getCenterSensorDistanceInCM();
        double sideSensorDistance = robot.getSideSensorDistanceInCM();
        int positionNumber = 0;

        if (centerSensorDistance == distanceOutOfRange && sideSensorDistance == distanceOutOfRange) {
            telemetry.addData("NOT DETECTED", "Object not detected by any sensor!");
            positionNumber = 3;
        }
        else if (centerSensorDistance != distanceOutOfRange) {
            telemetry.addData("DETECTED SIDE", "Object distance is %.0f CM", sideSensorDistance);
            positionNumber = 1;
        }
        else {
            telemetry.addData("DETECTED CENTER", "Object distance is %.0f CM", centerSensorDistance);
            positionNumber = 2;
        }
        telemetry.update();

        return positionNumber;
    }

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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;

/**
 * This 2023-2024 OpMode show a way to detect an image,
 * determine the position number it is in, then move
 * the robot to it.
 */
@TeleOp(name = "Detect image and move robot to it", group = "")
public class CenterstageAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware robot = new RobotHardware(this);
        robot.init();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {
            Recognition recognition = null;

            List<Recognition> recognitions = robot.getFreshTfodRecognitions();
            //Let's see if we see anything.
            if (recognitions == null || recognitions.isEmpty()) {
                telemetry.addData("", "I don't recognize anything.");
                telemetry.addData("Goodbye", "Ending Op Mode");
                telemetry.update();
            }
            //We should see no more than one recognition.
            else if (recognitions.size() > 1) {
                telemetry.addData("Too many", "I see %i recognitions", recognitions.size());
                telemetry.addData("Goodbye", "Ending Op Mode");
                telemetry.update();
            }
            //This recognitions list containing one recognition is juuuuust right.
            else {
                recognition = recognitions.get(0); //Get the first/only Recognition
                double recognizedCol = (recognition.getLeft() + recognition.getRight()) / 2;
                double recognizedRow = (recognition.getTop() + recognition.getBottom()) / 2;
                double recognizedWidth = Math.abs(recognition.getRight() - recognition.getLeft());
                double recognizedHeight = Math.abs(recognition.getTop() - recognition.getBottom());

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", recognizedRow, recognizedCol);
                telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", recognizedWidth, recognizedHeight);
                telemetry.update();

                int positionNumber = getPositionNumber(recognizedRow, recognizedCol);
                int driveLeftInches = 0;
                int driveRightInches = 0;

                switch (positionNumber) {
                    case 1:
                        driveLeftInches = -22;
                        driveRightInches = -22;
                        break;
                    case 2:
                        driveLeftInches = -23;
                        driveRightInches = -23;
                        break;
                    case 3:
                        driveLeftInches = -24;
                        driveRightInches = -24;
                        break;
                }

                robot.autoDriveRobot(driveLeftInches, driveRightInches);

                // Share the CPU.
                sleep(20);
            }
        }

        robot.shutDown();
    }   // end runOpMode()

    /**
     * TO-DO
     * Calculate a position number from recognition data.
     * @param recognizedRow
     * @param recognizedCol
     * @return int
     */
    private int getPositionNumber(double recognizedRow, double recognizedCol) {
        //Some sort of calculation to get a position number.
        return 42; //Secret of life.
    }
}   // end class

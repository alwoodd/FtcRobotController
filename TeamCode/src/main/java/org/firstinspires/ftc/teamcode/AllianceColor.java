package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

public enum AllianceColor {
    RED,
    BLUE,
    INSTANCE;

    /**
     * Standard way to choose an AllianceColor via a Driver Hub.
     * @param opMode LinearOpMode of the caller.
     * @return AllianceColor RED or BLUE
     */
    public AllianceColor selectRedOrBlue(LinearOpMode opMode) {
        Telemetry telemetry = opMode.telemetry;
        Gamepad gamepad = opMode.gamepad1;
        AllianceColor selectedColor = AllianceColor.RED;

        while (!opMode.isStopRequested()) {
            telemetry.addLine("Press Left Bumper to toggle between Red and Blue alliance.");
            telemetry.addLine("Press Right Bumper to confirm selection.");
            telemetry.addData(selectedColor.toString(), " currently selected");
            telemetry.update();

            if (gamepad.leftBumperWasPressed()) {
                if (selectedColor == AllianceColor.RED) {
                    selectedColor = AllianceColor.BLUE;
                }
                else {
                    selectedColor = AllianceColor.RED;
                }
            }
            else if (gamepad.rightBumperWasPressed()) {
                break;
            }
        }

        return selectedColor;
    }
}

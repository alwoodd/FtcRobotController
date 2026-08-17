package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class ColorLimelightMetrics extends OpMode {
    private Limelight3A limelight;
    private boolean pauseToggle = false;
    private int currentFeet = 1;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            incrementFeet();
        }

        telemetry.addData("Current Feet", currentFeet);

        if (gamepad1.yWasPressed()) {
            pauseToggle = !pauseToggle;
        }
        if (pauseToggle) return;

        LLResult llResult;

        llResult = limelight.getLatestResult();
        if (llResult.isValid()) {
            telemetry.addLine();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());

            //RobotLog.ii("Limelight", "Tx %.4f, Ty %4f, Ta %4f", llResult.getTx(), llResult.getTy(), llResult.getTa());
            //RobotLog.ii("Limelight", "Feet %d Ta %4f", currentFeet, llResult.getTa());
        }

    }

    @Override
    public void stop() {
        limelight.stop();
    }

    private void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private void incrementFeet() {
        currentFeet += 1;
        if (currentFeet > 4)
            currentFeet = 1;
    }
}

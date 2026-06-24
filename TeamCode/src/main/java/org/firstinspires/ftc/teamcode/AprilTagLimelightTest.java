package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConfiguration;

import java.util.List;

@Autonomous
public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;
    private Follower follower;

    @Override
    public void init() {
        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);
        follower = pedroPathConfiguration.getFollower();
        Pose startPose = new Pose(85.0, 8.5, Math.toRadians(90));
        follower.setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
    }

    @Override
    public void start() {
        limelight.start();
    }

    private LLResult lastValidResult;

    @Override
    public void loop() {
        calculateStartingPose();
    }

    private double llX = 0;
    private double llY = 0;
    private double llYaw = 0;
    private double pedroX = 0;
    private double pedroY = 0;
    private double pedroHeading = 0;

    private void calculateStartingPose() {
        String resultStatus = "";

        LLResult llResult = limelight.getLatestResult();

        if (llResult.isValid()) {
            resultStatus = "I see tag #" + getAllTagIds(llResult.getFiducialResults());
            Pose3D pose3D = llResult.getBotpose();
            Position position3D = pose3D.getPosition().toUnit(DistanceUnit.INCH);
            llX = position3D.x;
            llY = position3D.y;
            llYaw = pose3D.getOrientation().getYaw();

            Pose pedroPose = pose3DToPedroPose(pose3D);
            pedroX = pedroPose.getX();
            pedroY = pedroPose.getY();
            pedroHeading = pedroPose.getHeading();
        }

        telemetry.addLine(resultStatus);
        telemetry.addLine("LL Pose3D values");
        telemetry.addData("X (inches)", llX);
        telemetry.addData("Y (inches)", llY);
        telemetry.addData("Yaw (degrees)", llYaw);
        telemetry.addLine();
        telemetry.addLine("Proposed Pedro values");
        telemetry.addData("X (inches)", pedroX);
        telemetry.addData("Y (inches)", pedroY);
        telemetry.addData("Heading (degrees)", pedroHeading);
    }

    private Pose pose3DToPedroPose(Pose3D pose3D) {
        Position position3D = pose3D.getPosition().toUnit(DistanceUnit.INCH);
        double llX = position3D.x;
        double llY = position3D.y;
        double llYaw = pose3D.getOrientation().getYaw();

        double pX;
        double pY;
        double pHeading;

        if (llX <= 0) {
            pY = Math.abs(llX) + 72;
        }
        else {
            pY = Math.abs(llX - 72);
        }

        if (llY <= 0) {
            pX = 72 - Math.abs(llY);
        }
        else {
            pX = 72 + llY;
        }

        if (llYaw > 0) {
            llYaw -= 360;
        }

        pHeading = llYaw + 270;

        return new Pose(pX, pY, pHeading);
    }

    private String getAllTagIds(List<LLResultTypes.FiducialResult> fiducialResults) {
        StringBuilder sb = new StringBuilder();

        for (LLResultTypes.FiducialResult result : fiducialResults) {
            if (sb.length() > 0) {
                sb.append(", ");
            }
            sb.append(result.getFiducialId());
        }

        return sb.toString();
    }
}

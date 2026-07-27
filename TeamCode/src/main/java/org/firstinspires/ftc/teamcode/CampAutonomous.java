package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConfiguration;
import org.firstinspires.ftc.teamcode.teamPedroPathing.TeamPoses;
import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;
import org.lhssa.ftc.teamcode.pedroPathing.HeadingInterpolationType;
import org.lhssa.ftc.teamcode.pedroPathing.PedroMotion;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPathTelemetry;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPather;
import org.lhssa.ftc.teamcode.pedroPathing.PedroSleep;

import java.time.temporal.Temporal;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

@Autonomous
public class CampAutonomous extends LinearOpMode {
    private PedroPather pedroPather;
    private PedroMotion pedroMotion;
    private PedroPathTelemetry pedroPathTelemetry;
    private PedroSleep pedroSleep;
    private Pose startFlowersPose;

    private int currentAction = 0;
    private List<Integer> actionSteps;
    private Iterator<Integer> actionStep;

    private final double POLLEN_PICKUP_SPEED = .25;

    @Override
    public void runOpMode() throws InterruptedException {

        Follower follower = new PedroPathConfiguration(this).getFollower();
        pedroPather = new PedroPather(AllianceColor.BLUE, AllianceColor.BLUE);
        pedroMotion = new PedroMotion(follower);
        pedroSleep = new PedroSleep(follower);
        actionSteps = new ArrayList<>();

        initSetup();
        if (isStopRequested()) return;

        pedroPathTelemetry = new PedroPathTelemetry(telemetry, follower, AllianceColor.RED);
        actionStep = actionSteps.iterator();
        currentAction = actionStep.hasNext() ? actionStep.next() : 99;

        follower.setStartingPose(TeamPoses.startPose);

        waitForStart();

        boolean isDone = false;
        while (opModeIsActive() && !isDone) {
            follower.update();
            isDone = performActions();
        }
    }

    private boolean performActions() {
        Path path;
        boolean isDone = false;
        String pedroMessage = "Unknown";

        switch (currentAction) {
            case 0:
                pedroMessage = "Going to left side pollen";
                path = pedroPather.pathBetween(TeamPoses.startPose, TeamPoses.beforeStartLeftPollenPose, HeadingInterpolationType.LINEAR);
                pedroMotion.goPath(path);
                if (pedroMotion.isPathComplete()) {
                    currentAction = actionStep.next();
                }
                break;
            case 1:
                pedroMessage = "Picking up left pollen";
                //intakeOn
                path = pedroPather.pathBetween(TeamPoses.startLeftPollenPose, TeamPoses.endLeftPollenPose, HeadingInterpolationType.TANGENT);
                pedroMotion.goPath(path, POLLEN_PICKUP_SPEED);
                if (pedroMotion.isPathComplete()) {
                    //intakeOff
                    currentAction = actionStep.next();
                    startFlowersPose = TeamPoses.startLeftDepositPollenPose;
                }
                break;
            case 2:
                pedroMessage = "Going to right side pollen";
                path = pedroPather.pathBetween(TeamPoses.startPose, TeamPoses.startRightPollenPose);
                pedroMotion.goPath(path);
                if (pedroMotion.isPathComplete()) {
                    currentAction = actionStep.next();
                }
                break;
            case 3:
                pedroMessage = "Picking up right pollen";
                //intakeOn
                path = pedroPather.pathBetween(TeamPoses.startRightPollenPose, TeamPoses.endRightPollenPose, HeadingInterpolationType.TANGENT);
                pedroMotion.goPath(path, POLLEN_PICKUP_SPEED);
                if (pedroMotion.isPathComplete()) {
                    //intakeOff
                    currentAction = actionStep.next();
                    startFlowersPose = TeamPoses.endRightPollenPose;
                }
                break;
            case 4:
                pedroMessage = "Going to back corner pollen";
                path = pedroPather.pathBetween(TeamPoses.startPose, TeamPoses.startCornerPollenPose);
                pedroMotion.goPath(path);
                if (pedroMotion.isPathComplete()) {
                    currentAction = actionStep.next();
                }
                break;
            case 5:
                pedroMessage = "Picking up corner pollen";
                //intakeOn
                path = pedroPather.pathBetween(TeamPoses.startCornerPollenPose, TeamPoses.endCornerPollenPose, HeadingInterpolationType.TANGENT);
                pedroMotion.goPath(path, POLLEN_PICKUP_SPEED);
                if (pedroMotion.isPathComplete()) {
                    //intakeOff
                    currentAction = actionStep.next();
                    startFlowersPose = TeamPoses.endCornerPollenPose;
                }
                break;
            case 10:
                pedroMessage = "Going to flowers";
                path = pedroPather.pathBetween(startFlowersPose, TeamPoses.endDepositPollenPose, HeadingInterpolationType.LINEAR);
                pedroMotion.goPath(path);
                if (pedroMotion.isPathComplete()) {
                    depositPollen();
                    currentAction = actionStep.next();
                }
                break;
            case 99:
                pedroMessage = "Done";
                isDone = true;
                break;
        }

        pedroPathTelemetry.pathTelemetry(pedroMessage);
        return isDone;
    }

    private void depositPollen() {
        pedroPathTelemetry.pathTelemetry("Depositing pollen");
        pedroSleep.sleep(3000);
    }

    private void initSetup() {
        String gameConfig = "Unknown";
        int tagId;
        int priorTagId = -99;
        LimelightHardware limelightHardware = new LimelightHardware(this, 8);
        limelightHardware.beginSearch();

        while (opModeInInit()) {
            limelightHardware.update();
            Integer[] tagIds = limelightHardware.getTagIds();
            if (tagIds.length == 1) {
                tagId = tagIds[0];
                if (tagId != priorTagId) {
                    priorTagId = tagId;
                    switch (tagId) {
                        //Left side pollen
                        case 20:
                            actionSteps.clear();
                            actionSteps.add(0);
                            actionSteps.add(1);
                            actionSteps.add(10);
                            actionSteps.add(99);
                            gameConfig = "Left Side Pollen";
                            break;
                        //Right side pollen
                        case 24:
                            actionSteps.clear();
                            actionSteps.add(2);
                            actionSteps.add(3);
                            actionSteps.add(10);
                            actionSteps.add(99);
                            gameConfig = "Right Side Pollen";
                            break;
                        //Back right corner pollen
                        case 11:
                            actionSteps.clear();
                            actionSteps.add(4);
                            actionSteps.add(5);
                            actionSteps.add(10);
                            actionSteps.add(99);
                            gameConfig = "Back Right Corner Pollen";
                            break;
                        default:
                            actionSteps.clear();
                            actionSteps.add(99);
                            gameConfig = "Unknown";
                    }
                }
            }
            else {
                tagId = -99;
                actionSteps.clear();
                actionSteps.add(0);
                gameConfig = "Not seeing a tag";
            }
            telemetry.addData("Game configuration", gameConfig);
            if (tagId != -99) {
                telemetry.addData("AprilTag Id", tagId);
            }
            telemetry.update();
        }
        limelightHardware.endSearch();
    }
}

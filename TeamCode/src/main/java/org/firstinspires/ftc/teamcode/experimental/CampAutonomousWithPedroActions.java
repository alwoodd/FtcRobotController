package org.firstinspires.ftc.teamcode.experimental;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LimelightHardware;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConfiguration;
import org.firstinspires.ftc.teamcode.teamPedroPathing.TeamPoses;
import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;
import org.lhssa.ftc.teamcode.pedroPathing.HeadingInterpolationType;
import org.lhssa.ftc.teamcode.pedroPathing.PedroMotion;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPathTelemetry;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPather;
import org.lhssa.ftc.teamcode.pedroPathing.PedroSleep;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

@Autonomous
public class CampAutonomousWithPedroActions extends LinearOpMode {
    private PedroPather pedroPather;
    private PedroMotion pedroMotion;
    private PedroPathTelemetry pedroPathTelemetry;
    private PedroSleep pedroSleep;

    private List<PedroAction> actionSteps;

    private final double POLLEN_PICKUP_SPEED = .25;

    //private RobotHardware robot;

    private LimelightHardware llHardware;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = new PedroPathConfiguration(this).getFollower();
        pedroPather = new PedroPather(AllianceColor.BLUE, AllianceColor.BLUE);
        pedroMotion = new PedroMotion(follower);
        pedroSleep = new PedroSleep(follower);
        actionSteps = new ArrayList<>();
        //robot = new RobotHardware(this);
        llHardware = new LimelightHardware(this, 8);

        initSetup();
        if (isStopRequested()) return;

        pedroPathTelemetry = new PedroPathTelemetry(telemetry, follower, AllianceColor.RED);
        Iterator<PedroAction> actionStep = actionSteps.iterator();
        PedroAction currentAction = actionStep.hasNext() ? actionStep.next() : new PedroNoAction();

        follower.setStartingPose(TeamPoses.startPose);

        waitForStart();

        llHardware.beginSearch();

        while (opModeIsActive()) {
            follower.update();
            llHardware.update();
            currentAction.update();
            if (currentAction.isComplete()) {
                if (actionStep.hasNext()) {
                    currentAction = actionStep.next();
                    pedroPathTelemetry.pathTelemetry(currentAction.getDescription());
                }
                else {
                    break;
                }
            }
        }
    }

    private void depositPollen() {
        pedroPathTelemetry.pathTelemetry("Depositing pollen");
        //robot.raiseLift();
        //robot.flickBucket();
        //robot.lowerLift();
        pedroSleep.sleep(3000);
    }

    private void intakeOn() {
        pedroPathTelemetry.pathTelemetry("Intake On");
        pedroSleep.sleep(3000);
    }


    private void intakeOff() {
        pedroPathTelemetry.pathTelemetry("Intake Off");
        pedroSleep.sleep(3000);
    }


    private void initSetup() {
        String gameConfig = "Unknown";
        int tagId;
        int priorTagId = -99;
        llHardware.beginSearch();

        while (opModeInInit()) {
            llHardware.update();
            Integer[] tagIds = llHardware.getTagIds();
            if (tagIds.length == 1) {
                tagId = tagIds[0];
                if (tagId != priorTagId) {
                    priorTagId = tagId;
                    switch (tagId) {
                        //Left side pollen
                        case 20:
                            actionSteps.clear();
                            actionSteps.add(new PedroActionPath("Going to left side pollen",
                                    pedroPather.pathBetween(TeamPoses.startPose, TeamPoses.beforeStartLeftPollenPose,
                                    HeadingInterpolationType.LINEAR),
                                    pedroMotion));
                            actionSteps.add(new PedroActionWithRunnable("Picking up left pollen",
                                    pedroPather.pathBetween(TeamPoses.startLeftPollenPose, TeamPoses.endLeftPollenPose,
                                    HeadingInterpolationType.TANGENT),
                                    pedroMotion, POLLEN_PICKUP_SPEED, this::intakeOn));
                            actionSteps.add(new PedroActionWithRunnable("Going to flowers",
                                    pedroPather.pathBetween(TeamPoses.endLeftPollenPose, TeamPoses.endDepositPollenPose,
                                    HeadingInterpolationType.LINEAR),
                                    pedroMotion, this::depositPollen/*robot::releaseDrone*/));
                            actionSteps.add(new PedroActionGoToObject("Moving to pollen", follower, llHardware));
                            gameConfig = "Left Side Pollen";
                            break;
                        //Right side pollen
                        case 24:
                            actionSteps.clear();
                            gameConfig = "Right Side Pollen";
                            break;
                        //Back right corner pollen
                        case 11:
                            actionSteps.clear();
                            gameConfig = "Back Right Corner Pollen";
                            break;
                        default:
                            actionSteps.clear();
                            gameConfig = "Unknown";
                    }
                }
            }
            else {
                tagId = -99;
                actionSteps.clear();
                //actionSteps.add(0);
                gameConfig = "Not seeing a tag";
            }
            telemetry.addData("Game configuration", gameConfig);
            if (tagId != -99) {
                telemetry.addData("AprilTag Id", tagId);
            }
            telemetry.update();
        }
        llHardware.endSearch();
    }
}

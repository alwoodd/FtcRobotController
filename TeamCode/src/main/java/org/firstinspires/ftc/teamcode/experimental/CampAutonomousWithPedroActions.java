package org.firstinspires.ftc.teamcode.experimental;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LimelightHardware;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConfiguration;
import org.firstinspires.ftc.teamcode.teamPedroPathing.TeamPoses;
import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;
import org.lhssa.ftc.teamcode.pedroPathing.HeadingInterpolationType;
import org.lhssa.ftc.teamcode.pedroPathing.PedroMotion;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPathTelemetry;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPather;
import org.lhssa.ftc.teamcode.pedroPathing.PedroSleep;

import org.lhssa.ftc.teamcode.pedroActions.PedroAction;
import org.lhssa.ftc.teamcode.pedroActions.PedroActionPath;
import org.lhssa.ftc.teamcode.pedroActions.PedroActionManager;
import org.lhssa.ftc.teamcode.pedroActions.PedroActionWithRunnable;

@Autonomous
public class CampAutonomousWithPedroActions extends LinearOpMode {
    private PedroPather pedroPather;
    private PedroMotion pedroMotion;
    private PedroPathTelemetry pedroPathTelemetry;
    private PedroSleep pedroSleep;

    //private List<PedroAction> actionSteps;
    private PedroActionManager actionManager;

    private final double POLLEN_PICKUP_SPEED = .25;
    private final int APRILTAG_PIPELINE = 8;
    private final int POLLEN_PIPELINE = 9;

    //private RobotHardware robot;

    private LimelightHardware llHardware;

    @Override
    public void runOpMode() throws InterruptedException {

        Follower follower = new PedroPathConfiguration(this).getFollower();
        pedroPather = new PedroPather(AllianceColor.BLUE, AllianceColor.BLUE);
        pedroMotion = new PedroMotion(follower);
        pedroSleep = new PedroSleep(follower);
        //actionSteps = new ArrayList<>();
        actionManager = new PedroActionManager();
        //robot = new RobotHardware(this);

        llHardware = new LimelightHardware(this, APRILTAG_PIPELINE);
        initSetup();
        if (isStopRequested()) return;

        pedroPathTelemetry = new PedroPathTelemetry(telemetry, follower, AllianceColor.RED);
        //Iterator<PedroAction> actionStep = actionSteps.iterator();
        PedroAction currentAction = actionManager.next();// = actionStep.hasNext() ? actionStep.next() : new PedroNoAction();
        llHardware.setPipeLineNumber(POLLEN_PIPELINE);
        pedroPathTelemetry.pathTelemetry(currentAction.getDescription());
        follower.setStartingPose(TeamPoses.startPose);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            llHardware.update();
            currentAction.update();
            if (currentAction.isComplete()) {
                if (actionManager.hasNext()) {
                    currentAction = actionManager.next();
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
        pedroSleep.sleep(2000);
    }

    private void intakeOn() {
        pedroPathTelemetry.pathTelemetry("Intake On");
        pedroSleep.sleep(2000);
    }


    private void intakeOff() {
        pedroPathTelemetry.pathTelemetry("Intake Off");
        pedroSleep.sleep(2000);
    }


    private void initSetup() {
        String gameConfig = "Unknown";
        int tagId = -99;
        llHardware.beginSearch();

        while (opModeInInit()) {
            llHardware.update();
            Integer[] tagIds = llHardware.getTagIds();
            if (tagIds.length == 1) {
                tagId = tagIds[0];
                    switch (tagId) {
                        case 20:
                            gameConfig = "Left Side Pollen";
                            break;
                        case 24:
                            gameConfig = "Right Side Pollen";
                            break;
                        case 11:
                            gameConfig = "Back Right Corner Pollen";
                            break;
                        default:
                            gameConfig = "Unknown";
                    }
            }
            else {
                tagId = -99;
                gameConfig = "Not seeing a tag";
            }

            telemetry.addData("Game configuration", gameConfig);
            if (tagId != -99) {
                telemetry.addData("AprilTag Id", tagId);
            }
            telemetry.update();
        }

        llHardware.endSearch();
        switch (tagId) {
            case 20:
                actionManager.add(new PedroActionPath("Going to left side pollen",
                    pedroPather.pathBetween(TeamPoses.startPose, TeamPoses.beforeStartLeftPollenPose,
                    HeadingInterpolationType.LINEAR),
                    pedroMotion));
                actionManager.add(new PedroActionWithRunnable("Picking up left pollen",
                    pedroPather.pathBetween(TeamPoses.startLeftPollenPose, TeamPoses.endLeftPollenPose,
                    HeadingInterpolationType.TANGENT),
                    pedroMotion, POLLEN_PICKUP_SPEED, this::intakeOn));
                actionManager.add(new PedroActionWithRunnable("Going to flowers",
                    pedroPather.pathBetween(TeamPoses.endLeftPollenPose, TeamPoses.endDepositPollenPose,
                    HeadingInterpolationType.LINEAR),
                    pedroMotion, this::depositPollen/*robot::releaseDrone*/));
                break;
            case 24:
            case 11:
        }
    }
}

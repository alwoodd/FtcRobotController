package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.lhssa.ftc.teamcode.pedroActions.PedroActionRunUntilComplete;
import org.lhssa.ftc.teamcode.pedroActions.PedroTimerAction;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConfiguration;
import org.firstinspires.ftc.teamcode.teamPedroPathing.TeamPoses;
import org.lhssa.ftc.teamcode.pedroActions.PedroAction;
import org.lhssa.ftc.teamcode.pedroActions.PedroActionManager;
import org.lhssa.ftc.teamcode.pedroActions.PedroActionPath;
import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;
import org.lhssa.ftc.teamcode.pedroPathing.HeadingInterpolationType;
import org.lhssa.ftc.teamcode.pedroPathing.PedroMotion;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPathTelemetry;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPather;

/*
 * This OpMode demonstrates how to use pedroActions and pedroPathing from the PedroLib module.
 */
@Autonomous
public class BioBuzzAutonomous extends OpMode {
    private AllianceColor allianceColor = AllianceColor.BLUE;
    private FrontBackLocation frontBackLocation = FrontBackLocation.BACK;
    private PedroActionManager actionManager;
    private PedroPather pedroPather;
    private PedroMotion pedroMotion;
    private PedroAction currentAction;
    private PedroPathTelemetry pedroTelemetry;
    private Follower follower;
    private LimelightHardware limelight;
    private final int RED_LIMELIGHT_PIPELINE = 3;
    private final int BLUE_LIMELIGHT_PIPELINE = 2;
    private Timer shootTimer; //Emulates using a shooter

    //All of these PedroActions are created once, then used in more than one place.
    private PedroActionRunUntilComplete shooterAction;
    private PedroActionRunUntilComplete waitForHiveDownAction;
    private PedroActionPath parkAction;
    private PedroTimerAction timeToParkAction;
    private final double MAX_SECONDS_BEFORE_PARK = 25;

    @Override
    public void init() {
        follower = new PedroPathConfiguration(this).getFollower();
        actionManager = new PedroActionManager();
        pedroMotion = new PedroMotion(follower);
        shootTimer = new Timer();

        shooterAction = new PedroActionRunUntilComplete("Shooting...", this::shootUntilComplete);
        waitForHiveDownAction = new PedroActionRunUntilComplete("Waiting for Hive Down", this::waitUntilHiveDown);
        timeToParkAction = new PedroTimerAction("Time to park!", MAX_SECONDS_BEFORE_PARK);
    }

    public void init_loop() {
        initSetup();
    }

    public void start() {
        pedroPather = new PedroPather(TeamPoses.canonicalColor, allianceColor, PedroPather::FlipRoutineAlliancesOnOppositeSides);
        follower.setPose(pedroPather.normalizePose(TeamPoses.startBiobuzz));

        pedroTelemetry = new PedroPathTelemetry(telemetry, follower, allianceColor);
        limelight = new LimelightHardware(this, allianceColor == AllianceColor.BLUE ? BLUE_LIMELIGHT_PIPELINE : RED_LIMELIGHT_PIPELINE);
        parkAction = new PedroActionPath("Parking",
                pedroPather.pathBetween(TeamPoses.startBiobuzz, TeamPoses.parkBiobuzz, HeadingInterpolationType.TANGENT), pedroMotion);

        actionSetup();

        //Prime currentAction
        currentAction = actionManager.next();
        pedroTelemetry.pathTelemetry(currentAction.getDescription());
    }

    @Override
    public void loop() {
        follower.update();

        /*
         * This PedroTimerAction is run on every loop.
         * Once it isComplete(), ignoreRemainingActions() and park.
         */
        timeToParkAction.update();
        if (timeToParkAction.isComplete()) {
            actionManager.ignoreRemainingActions();
            currentAction = parkAction;
            pedroTelemetry.pathTelemetry(currentAction.getDescription());
        }

        currentAction.update();
        if (currentAction.isComplete()) {
            if (actionManager.hasNext()) {
                currentAction = actionManager.next();
                pedroTelemetry.pathTelemetry(currentAction.getDescription());
            }
            else {
                requestOpModeStop(); //Stop when no more actions.
            }
        }
    }

    /**
     * This is called repeatedly by shooterAction until the shooter reports isComplete().
     * @return true when shooting is complete
     */
    private boolean shootUntilComplete() {
        boolean isComplete;

        if (!shooterAction.isStarted()) {
                //shoot motor on:
                shootTimer.reset();
                shooterAction.start();
        }

        //shoot motor off:
        isComplete = shootTimer.milliseconds() > 3000;

        return isComplete;
    }

    /**
     * This is called repeatedly by waitForHiveDownAction until the hive is down.
     * @return true when the limelight detects the hive is up
     */
    private boolean waitUntilHiveDown() {
        boolean isHiveDown = false;

        if (!waitForHiveDownAction.isStarted()) {
           limelight.beginSearch();
           waitForHiveDownAction.start();
        }

        limelight.update();
        if (limelight.getTagIds().length > 0) {
            limelight.endSearch();
            isHiveDown = true;
        }
        return isHiveDown;
    }

    /**
     * Configure OpMode for the match
     */
    private void initSetup() {
        telemetry.addLine("Press X to toggle Alliance");
        telemetry.addData("Current alliance", allianceColor.toString());
        telemetry.addLine("Press Y to toggle Side");
        telemetry.addData("Current side", frontBackLocation.toString());
        telemetry.update();

        allianceColor = AllianceColor.INSTANCE.toggleColor(gamepad1.xWasPressed(), allianceColor);
        frontBackLocation = FrontBackLocation.INSTANCE.toggleLocation(gamepad1.yWasPressed(), frontBackLocation);
    }

    /**
     * Add all the PedroActions in the order they should run to the PedroActionManager.
     */
    private void actionSetup() {
        if (allianceColor == AllianceColor.BLUE) {
            if (frontBackLocation == FrontBackLocation.BACK) {
                actionManager.add(this.shooterAction);
                addCommonActions();
            }
            else  {
                actionManager.add(this.waitForHiveDownAction);
                actionManager.add(this.shooterAction);
                addCommonActions();
            }
        }
        //AllianceColor.RED
        else {
            if (frontBackLocation == FrontBackLocation.FRONT) {
                actionManager.add(this.shooterAction);
                addCommonActions();
            }
            else {
                actionManager.add(this.waitForHiveDownAction);
                actionManager.add(this.shooterAction);
                addCommonActions();
            }
        }
    }

    /**
     * Common sequence of actions for all matches.
     */
    private void addCommonActions() {
        actionManager.add(new PedroActionPath("Going to garden",
                pedroPather.pathBetween(TeamPoses.startBiobuzz, TeamPoses.beginGardenPickup, HeadingInterpolationType.TANGENT),
                pedroMotion));
        actionManager.add(new PedroActionPath("Picking up pollen from garden",
                pedroPather.pathBetween(TeamPoses.beginGardenPickup, TeamPoses.endGardenPickup, HeadingInterpolationType.TANGENT),
                pedroMotion,.1));
        actionManager.add(new PedroActionPath("Back to shoot position",
                pedroPather.pathBetween(TeamPoses.endGardenPickup, TeamPoses.startBiobuzz, HeadingInterpolationType.LINEAR),
                pedroMotion));
        actionManager.add(this.waitForHiveDownAction);
        actionManager.add(this.shooterAction);
        actionManager.add(this.parkAction);
    }
}
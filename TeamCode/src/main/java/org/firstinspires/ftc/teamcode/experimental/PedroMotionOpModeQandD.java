package org.firstinspires.ftc.teamcode.experimental;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConfiguration;
import org.firstinspires.ftc.teamcode.teamPedroPathing.TeamPoses;
import org.lhssa.ftc.teamcode.pedroActions.PedroAction;
import org.lhssa.ftc.teamcode.pedroActions.PedroActionManager;
import org.lhssa.ftc.teamcode.pedroActions.PedroActionPath;
import org.lhssa.ftc.teamcode.pedroActions.PedroActionWithRunnable;
import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;
import org.lhssa.ftc.teamcode.pedroPathing.PedroMotion;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPathTelemetry;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPather;
import org.lhssa.ftc.teamcode.pedroPathing.PedroSleep;

@Autonomous
@Disabled
public class PedroMotionOpModeQandD extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = new PedroPathConfiguration(this).getFollower();
        PedroPather pedroPather = new PedroPather(AllianceColor.BLUE, AllianceColor.BLUE);
        PedroMotion pedroMotion = new PedroMotion(follower);
        PedroPathTelemetry pedroPathTelemetry = new PedroPathTelemetry(telemetry, follower, AllianceColor.RED);
        PedroSleep pedroSleep = new PedroSleep(follower);
        PedroActionManager pedroActionManager = new PedroActionManager();

/*
        PedroAction pedroAction = new PedroActionWithRunnable("blah",
                pedroPather.pathBetween(TeamPoses.startPose, TeamPoses.beforeStartLeftPollenPose),
                pedroMotion,
                this::doThing);
*/
//        pedroActionManager.add(pedroAction);
        PedroAction pedroAction = new PedroActionPath("blah",
                pedroPather.pathBetween(TeamPoses.startPose, TeamPoses.beforeStartLeftPollenPose),
                pedroMotion);

        pedroPathTelemetry.pathTelemetry(pedroAction.getDescription());
        follower.setPose(TeamPoses.startPose);

/*
        follower.update();
        follower.follow(pedroPather.pathBetween(TeamPoses.startPose, TeamPoses.beforeStartLeftPollenPose));
*/
        //pedroAction = pedroActionManager.next();

        waitForStart();

        while (opModeIsActive() /*&& follower.isBusy()*/) {
            follower.update();
            pedroAction.update();
            RobotLog.ii("PedroMotionOpModeQandD", "X: %.2f, Y: %.2f, Heading: %.2f", follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
            if (pedroAction.isComplete()) {
                RobotLog.ii("PedroMotionOpModeQandD", "pedroAction.isComplete");
//                telemetry.addLine("isComplete()");
//                if (pedroActionManager.hasNext()) {
//                    telemetry.addLine("...and hasNext()");
//                    pedroAction = pedroActionManager.next();
//                }
//                else {
                    break;
//                }
            }
/*
            Path path = pedroPather.pathBetween(TeamPoses.startPose, TeamPoses.beforeStartLeftPollenPose);
            pedroMotion.goPath(path);
            if (pedroMotion.isPathComplete()) break;
*/
        }

        RobotLog.ii("PedroMotionOpModeQandD", "while loop exited");
        telemetry.addLine("while loop exited");
        telemetry.update();
        pedroSleep.sleep(3000);
    }

    private void doThing() {
        //sleep(500);
    }
}

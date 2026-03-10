package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;

/**
 * Define all the team's Poses here. The Poses should be either for RED or BLUE, and the
 * canonicalColor set accordingly.
 */
public class TeamPoses {
    public static AllianceColor canonicalColor = AllianceColor.RED;

    public static Pose frontWallStartingPose = new Pose(85.0, 8.5, Math.toRadians(90));
    public static Pose backWallStartingPose = new Pose(121.3, 125.3, Math.toRadians(-142));
    public static Pose backGoalShootPose = new Pose(108.4, 120.3, Math.toRadians(28));
    public static Pose frontGoalShootPose = new Pose(85, 8.5, Math.toRadians(70));

    public static Pose startAudienceBallPickupPose = new Pose(108.7, 35.5, Math.toRadians(180));
    public static Pose endAudienceBallPickupPose = new Pose(119.9, 35.5, Math.toRadians(180));

    public static Pose startMiddleBallPickupPose = new Pose(108.7, 59.7, Math.toRadians(180));
    public static Pose endMiddleBallPickupPose = new Pose(119.9, 59.7, Math.toRadians(180));

    public static Pose startGoalBallPickupPose = new Pose(108.7, 83.5, Math.toRadians(180));
    public static Pose endGoalBallPickupPose = new Pose(119.9, 83.5, Math.toRadians(180));

    public static Pose parkPose = new Pose(38.8, 33.2, Math.toRadians(180));
}

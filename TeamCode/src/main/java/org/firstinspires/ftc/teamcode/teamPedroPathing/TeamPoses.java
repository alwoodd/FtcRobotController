package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.geometry.Pose;

import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;

/**
 * Define all the team's Poses here. The Poses should be either for RED or BLUE, and the
 * canonicalColor set accordingly.
 */
public class TeamPoses {
    public static AllianceColor canonicalColor = AllianceColor.BLUE;

    public static Pose startPose = new Pose(56, 8, Math.toRadians(90));
    public static Pose beforeStartLeftPollenPose = new Pose(24,46, Math.toRadians(130));
    public static Pose startLeftPollenPose = new Pose(24, 46, Math.toRadians(90));
    public static Pose endLeftPollenPose = new Pose(24, 80, Math.toRadians(90));
    public static Pose startLeftDepositPollenPose = new Pose(24, 80, Math.toRadians(54));
    public static Pose endDepositPollenPose = new Pose(69, 125, Math.toRadians(90));

    public static Pose startRightPollenPose = new Pose(100, 59, 0);
    public static Pose endRightPollenPose = new Pose(120, 59, 0);

    public static Pose startCornerPollenPose = new Pose(120, 8, 0);
    public static Pose endCornerPollenPose = new Pose(130, 8, 0);

    /***********************************************************************************************/
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

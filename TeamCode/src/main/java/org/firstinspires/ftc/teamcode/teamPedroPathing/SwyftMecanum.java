package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Extends Mecanum and adds explicit setting of each motor's RunMode.
 * Swyft motors that use RUN_USING_ENCODER will randomly brake when power goes to 0.
 * Therefore we want them to explicitly be RUN_WITHOUT_ENCODER. Unfortunately, the Mecanum
 * class doesn't expose or set RunMode one way or the other, so this class extends Mecanum
 * with that ability.
 * Since we're using Pinpoint odometry, we don't need, or want, RUN_USING_ENCODER anyway.
 */
public class SwyftMecanum extends Mecanum {
    /**
     * @param hardwareMap      this is the HardwareMap object that contains the motors and other hardware
     * @param mecanumConstants this is the MecanumConstants object that contains the names of the motors and directions etc.
     */
    public SwyftMecanum(HardwareMap hardwareMap, MecanumConstants mecanumConstants) {
        super(hardwareMap, mecanumConstants);

        for (DcMotor motor : this.getMotors()) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}

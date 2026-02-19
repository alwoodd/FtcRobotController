package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.follower.Follower;

//Design crutch. See Continuous OpMode for better design.
public class PedroSleep {
    private final Follower follower;
    private final long updateFrequency = 250;

    public PedroSleep(Follower follower) {
        this.follower = follower;
    }

    public void sleep(long milliseconds) {
        for (long m = 0; m < milliseconds; m += updateFrequency) {
            sleepImpl();
            follower.update();
        }
    }

    private void sleepImpl() {
        try {
            Thread.sleep(updateFrequency);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

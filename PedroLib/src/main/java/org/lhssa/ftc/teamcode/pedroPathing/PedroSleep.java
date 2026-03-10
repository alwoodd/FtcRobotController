package org.lhssa.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;

//Design crutch. See Continuous OpMode for better design.
public class PedroSleep {
    private final Follower follower;
    private long updateFrequency = 250;

    public PedroSleep(Follower follower, long updateFrequency) {
        this.follower = follower;
        this.updateFrequency = updateFrequency;
    }

    public PedroSleep(Follower follower) {
        this.follower = follower;
    }

    public void sleep(long milliseconds) {
        for (long m = 0; m < milliseconds; m += updateFrequency) {
            sleepImpl(updateFrequency);
            follower.update();
        }
    }

    private void sleepImpl(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

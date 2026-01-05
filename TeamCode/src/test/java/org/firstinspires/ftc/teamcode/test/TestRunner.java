package org.firstinspires.ftc.teamcode.test;

import static org.junit.Assert.assertTrue;

import com.pedropathing.follower.Follower;

import org.junit.Test;

public class TestRunner {
    @Test
    public void run() {
        TestPedroPathHardware pedroPathHardware = new TestPedroPathHardware();
        Follower follower = pedroPathHardware.getFollower();
        assertTrue(true);
    }
}

package org.firstinspires.ftc.teamcode.experimental;

import com.pedropathing.paths.Path;

import org.lhssa.ftc.teamcode.pedroPathing.PedroMotion;

/**
 * PedroAction that follows the passed Path, then runs the passed callback
 * at Path completion.
 */
public class PedroActionWithRunnable implements PedroAction{
    private final Path path;
    private final Runnable callBack;
    private final PedroMotion pedroMotion;
    private final String description;
    private final double speed;

    private boolean isComplete = false;


    /**
     * Constructor that uses the Follower's max power.
     * @param description Specific action(s) being performed by this instance.
     * @param path Path to follow
     * @param pedroMotion PedroMotion instance
     * @param callBack Runnable class or method invoked after Path is complete.
     */
    public PedroActionWithRunnable(String description, Path path, PedroMotion pedroMotion, Runnable callBack) {
        this.path = path;
        this.callBack = callBack;
        this.pedroMotion = pedroMotion;
        this.description = description;
        this.speed = -999;
    }

    /**
     * Constructor that uses the passed speed.
     * @param description Specific action(s) being performed by this instance.
     * @param path Path to follow
     * @param pedroMotion PedroMotion instance
     * @param speed Speed Follower will use
     * @param callBack Runnable class or method invoked after Path is complete.
     */
    public PedroActionWithRunnable(String description, Path path, PedroMotion pedroMotion, double speed, Runnable callBack) {
        this.path = path;
        this.callBack = callBack;
        this.pedroMotion = pedroMotion;
        this.description = description;
        this.speed = speed;
    }

    @Override
    public void update() {
        if (isSpeedValid()) {
            pedroMotion.goPath(path, speed);
        }
        else {
            pedroMotion.goPath(path);
        }

        if (pedroMotion.isPathComplete()) {
            callBack.run();
            isComplete = true;
        }
    }

    @Override
    public boolean isComplete() {
        return isComplete;
    }

    @Override
    public String getDescription() {
        return description;
    }

    private boolean isSpeedValid() {
        return (speed != -999);
    }
}

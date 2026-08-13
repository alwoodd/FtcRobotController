package org.firstinspires.ftc.teamcode.experimental;

import com.pedropathing.paths.Path;

import org.lhssa.ftc.teamcode.pedroPathing.PedroMotion;

/**
 * PedroAction that just follows the passed Path.
 */
public class PedroActionPath implements PedroAction{
    private final Path path;
    private final PedroMotion pedroMotion;
    private final String description;
    private final double speed;

    /**
     * Constructor that uses the Follower's max power.
     * @param description Specific action(s) being performed by this instance.
     * @param path Path to follow
     * @param pedroMotion PedroMotion instance
     */
    public PedroActionPath(String description, Path path, PedroMotion pedroMotion) {
        this.path = path;
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
     */
    public PedroActionPath(String description, Path path, PedroMotion pedroMotion, double speed) {
        this.path = path;
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
    }

    @Override
    public boolean isComplete() {
        return pedroMotion.isPathComplete();
    }

    @Override
    public String getDescription() {
        return description;
    }

    private boolean isSpeedValid() {
        return (speed != -999);
    }
}

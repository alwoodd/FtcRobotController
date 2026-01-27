package org.firstinspires.ftc.teamcode.teamPedroPathing;

import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.follower.Follower;


public class PedroPathFlipper {
    private final Follower follower;

    public PedroPathFlipper(Follower follower) {
        this.follower = follower;
    }

    /**
     * Take a PathChain containing FlippablePaths, calls FlippablePath.flipRightToLeft() on each one,
     * then returns a new PathChain of FlippablePaths.
     * @param pathChain PathChain containing FlippablePaths
     * @return PathChain of FlippablePaths
     * @throws UnsupportedOperationException if any Path is not a FlippablePath.
     */
    public PathChain flipPathChain(PathChain pathChain) {
        Path currentPath;
        FlippablePath currentFlippablePath;
        PathBuilder builder = this.follower.pathBuilder();

        //For each Path in the passed pathChain
        for (int i = 0; i < pathChain.size(); i++) {
            currentPath = pathChain.getPath(i);

            if (!(currentPath instanceof FlippablePath)) {
                throw new UnsupportedOperationException("PathChain must contain instances of FlippablePath.");
            }

            currentFlippablePath = (FlippablePath) currentPath;
            builder.addPath(currentFlippablePath.flipPath());
        }
        //return a new PathChain
        return builder.build();
    }
}

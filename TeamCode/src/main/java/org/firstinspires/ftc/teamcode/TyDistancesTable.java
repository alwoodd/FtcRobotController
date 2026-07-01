package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * TyDistancesTable
 *
 * Implements interpolated distance calculations based on ranges of pre-measured upper and lower distances
 * and their associated Ty angle values.
 * Add Ty-distance values to an instance one at a time using addTyDistance(), and/or all at once passing an
 * array of TyDistance objects using addTyDistances().
 * distanceFor() will return -1 if fewer than 2 Ty-distances are added.
 */
public class TyDistancesTable {
    private final List<TyDistance> tyDistances = new ArrayList<>();
    private boolean isAddSinceSort = true;

    /**
     * Adds a Ty-Distance pair.
     * @param tY Ty angle for this distance
     * @param distance Distance for this Ty angle
     */
    public void addTyDistance(double tY, double distance) {
        this.tyDistances.add(new TyDistance(tY, distance));
        isAddSinceSort = true;
    }

    /**
     * Adds an array of TyDistances all in one go.
     * @param tyDistancesArray TyDistance[]
     */
    public void addTyDistances(TyDistance[] tyDistancesArray) {
        this.tyDistances.addAll(Arrays.asList(tyDistancesArray));
        isAddSinceSort = true;
    }

    /**
     * Returns a calculated distance for the passed tY angle value.
     * It is expected that all Ty-distance values have first been added.
     * WARNINGS:
     *   Returns -1 if fewer than 2 Ty-distances have been added.
     *   A non-zero distance will be returned when Ty is 0 (IOW object not detected).
     * @param tY Ty angle
     * @return distance for the passed Ty
     */
    public double distanceFor(double tY) {
        double distance = -1;

        //We need at least 2 TyDistances for this to work.
        if (tyDistances.size() > 1) {

            /*
             * The camera is assumed to be higher than the target object, so all Ty angles are negative.
             * Make sure List is sorted from greatest to least angle (which is also highest to lowest distance).
             */
            if (isAddSinceSort) {
                this.tyDistances.sort(Comparator.comparingDouble(TyDistance::getTy).reversed());
                isAddSinceSort = false;
            }

            TyDistance upper = null;
            TyDistance lower = null;

            //Find the TyDistances that bound the given tY.
            for (int i = 0; i < tyDistances.size(); i++) {
                lower = tyDistances.get(i);
                /*
                 * If tY is > our table's first bound, then clamp the range so upper is the table's first entry,
                 * and lower is the second entry.
                 */
                if (tY >= lower.getTy()) {
                    if (i == 0) {
                        upper = lower;
                        lower = tyDistances.get(1);
                    }
                    else {
                        upper = tyDistances.get(i - 1);
                    }
                    break;
                }
            }

            /*
             * If upper never got set, then our Ty was greater than the highest angle in the table
             * Clamp the range so upper is the table's second-to-the-last entry,
             * and lower is the last entry.
             */
            if (upper == null ) {
                lower = tyDistances.get(tyDistances.size() - 1);
                upper = tyDistances.get(tyDistances.size() - 2);
            }

            distance = calculateDistance(tY, lower, upper);
        }

        return distance;
    }

    /**
     * Implements the distance interpolation calculation.
     * @param tY Ty angle we want an interpolated distance for
     * @param lower TyDistance
     * @param upper TyDistance
     * @return interpolated distance
     */
    private double calculateDistance(double tY, TyDistance lower, TyDistance upper) {
        //How far away is this tY from the lower baseline?
        double angleDistance = tY - lower.getTy();
        //What is the total span of distance in this bracket?
        double distanceSpan = upper.getDistance() - lower.getDistance();
        //What is the total span of angles in this bracket?
        double angleSpan = upper.getTy() - lower.getTy();
        //Scale the progress by the angleSpan.
        double progress = angleDistance / angleSpan;
        //Multiply progress by the distance span
        progress *= distanceSpan;
        //Add progress to baseline to get distance.
        return lower.getDistance() + progress;
    }

    /**
     * TyDistance
     * Implements a simple Ty-Distance pair.
     */
    public static class TyDistance {
        private final double tY;
        private final double distance;

        public TyDistance(double tY, double distance) {
            this.tY = tY;
            this.distance = distance;
        }

        public double getTy() {return this.tY;}
        public double getDistance() {return this.distance;}
    }
}
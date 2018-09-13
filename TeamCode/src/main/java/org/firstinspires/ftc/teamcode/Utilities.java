package org.firstinspires.ftc.teamcode;

/**
 * Created by Sarthak on 10/6/2017.
 */

public class Utilities {
    public static double round2D(double input) {
        input *= 100;
        input = Math.round(input);
        return input / 100;
    }

}

package org.firstinspires.ftc.teamcode.subsystems.latching;

/**
 * Created by Sarthak on 9/15/2018.
 */
public interface ILatching {

    /**
     * Releases the mechanism from the lander, begins the landing process
     */
    void release();

    /**
     * Once secured, lifts the robot off the playing field into a hanging position
     */
    void hold();

    /**
     * Secures the latching mechanism to the hook on the lander, preparing it to lift
     */
    void hook();

}

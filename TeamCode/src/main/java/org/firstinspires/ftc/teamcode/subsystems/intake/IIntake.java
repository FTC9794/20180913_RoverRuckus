package org.firstinspires.ftc.teamcode.subsystems.intake;

/**
 * Created by Sarthak on 9/14/2018.
 */
public interface IIntake {

    /**
     * Starts to intake minerals into the robot
     */
    void intake();

    /**
     * Starts to eject minerals out of the robot
     */
    void outtake();

    /**
     * Stops the intake mechanism, sets power to zero
     */
    void stop();

}

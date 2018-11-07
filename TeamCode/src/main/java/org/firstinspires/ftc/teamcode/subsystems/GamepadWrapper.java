package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Sarthak on 6/29/2017.
 */

public class GamepadWrapper {

    private Gamepad gamepad;

    /**
     * Constructor for gamepad wrapper class
     * @param gamepad gamepad object to be passed in
     */
    public GamepadWrapper(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    /**
     * Calculates the distance that the joystick has been pushed
     * @return the distance as a double
     */
    public double getDistanceFromCenterLeft(){
        return Math.sqrt(Math.pow(this.leftStickX(), 2)+Math.pow(this.leftStickY(), 2));
    }

    /**
     * Get the angle that the left stick has been pushed at
     * @return return the angle as a double
     */
    public double getAngleLeftStick() {
        return Math.toDegrees(Math.atan2(this.leftStickX(), this.leftStickY()));

    }

    /**
     * Get the distance from the center of the right joystic
     * @return the distance from the center as a double
     */
    public double getDistanceFromCenterRight(){
        return Math.sqrt(Math.pow(this.rightStickX(), 2)+Math.pow(this.rightStickY(), 2));
    }

    /**
     * Get the angle that the right stick has been pushed at
     * @return return the angle as a double
     */
    public double getAngleRightStick() {
        return Math.toDegrees(Math.atan2(this.rightStickX(), this.rightStickY()));

    }

    /**
     * Determines whether the a button is pressed
     * @return true if the button is pressed, false if not
     */
    public boolean a(){
        return gamepad.a;
    }

    /**
     * Determines whether the b button is pressed
     * @return true if the button is pressed, false if not
     */
    public boolean b(){
        return gamepad.b;
    }

    /**
     * Determines whether the x button is pressed
     * @return true if the button is pressed, false if not
     */
    public boolean x(){
        return gamepad.x;
    }

    /**
     * Determines whether the y button is pressed
     * @return true if the button is pressed, false if not
     */
    public boolean y(){
        return gamepad.y;
    }

    /**
     * Determines whether the d-pad up button is pressed
     * @return true if the button is pressed, false if not
     */
    public boolean dpadUp(){
        return gamepad.dpad_up;
    }

    /**
     * Determines whether the d-pad down button is pressed
     * @return true if the button is pressed, false if not
     */
    public boolean dpadDown(){
        return gamepad.dpad_down;
    }

    /**
     * Determines whether the d-pad left button is pressed
     * @return true if the button is pressed, false if not
     */
    public boolean dpadLeft(){
        return gamepad.dpad_left;
    }

    /**
     * Determines whether the d-pad right button is pressed
     * @return true if the button is pressed, false if not
     */
    public boolean dpadRight(){
        return gamepad.dpad_right;
    }

    /**
     * Returns the amount that the left trigger has been pressed
     * @return the value of the left trigger as a double
     */
    public double leftTrigger(){
        return gamepad.left_trigger;
    }

    /**
     * Returns the amount that the right trigger has been pressed
     * @return the value of the right trigger as a double
     */
    public double rightTrigger(){
        return gamepad.right_trigger;
    }

    /**
     * Determines whether the left bumper is pressed
     * @return true if the button is pressed, false if not
     */
    public boolean leftBumper(){
        return gamepad.left_bumper;
    }

    /**
     * Determines whether the right bumper is pressed
     * @return true if the button is pressed, false if not
     */
    public boolean rightBumper(){
        return gamepad.right_bumper;
    }

    /**
     * Determines how much the right stick has been pushed on the x axis
     * @return the distance on the x axis as a double
     */
    public double rightStickX(){
        return gamepad.right_stick_x;
    }

    /**
     * Determines how much the right stick has been pushed on the y axis
     * @return the distance on the y axis as a double
     */
    public double rightStickY(){
        return -gamepad.right_stick_y;
    }

    /**
     * Determines how much the left stick has been pushed on the x axis
     * @return the distance on the x axis as a double
     */
    public double leftStickX(){
        return gamepad.left_stick_x;
    }

    /**
     * Determines how much the left stick has been pushed on the y axis
     * @return the distance on the y axis as a double
     */

    public double leftStickY(){
        return -gamepad.left_stick_y;
    }
}

package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.firstinspires.ftc.teamcode.Enums.Direction;

/**
 * Created by Sarthak on 8/1/2018.
 */
public interface IDrivetrain {

    /**
     * Moves the robot in any lateral direction while maintaining a certain orientation.
     * @param currentPosition The current position of robot in any unit eg. encoder counts, sensor distances...
     * @param targetPosition The target position of the robot in the same unit as current position
     * @param rampDownTargetPosition Position at which the robot will start ramping down
     * @param rampUpTargetPosition Position at which the robot will stop ramping up in power
     * @param maxPower The maximum power the robot will move at, ranging from 0.0 to 1.0
     * @param lowPower The lowest power the robot will move at, ranging from 0.0 to 1.0
     * @param moveAngle The angle at which the robot will move in the frame of reference of the starting position
     * @param PIDGain Three gains to control PID feedback loop for Orientation correction
     * @param endOrientationAngle The Direction the robot is facing
     * @return true if the motion is complete, false is the motion is ongoing
     */
    boolean move(double currentPosition, double targetPosition, double rampDownTargetPosition, double rampUpTargetPosition, double rampDownEnd, double maxPower, double lowPower, double moveAngle, double[] PIDGain, double endOrientationAngle, double allowableDistanceError, double correctiontime);

    /**
     * Pivots the robot to a desired angle. It uses a proportional control loop to maintain the robot's speed
     * @param desiredAngle The angle to which to pivot to
     * @param rampDownAngle The angle at which to start slowing down
     * @param maxPower The max power to pivot at
     * @param minPower The min power to pivot at
     * @param correctionTime The amount of time to spend correcting to stay within the desired range
     * @return true if the motion is complete, false is the motion is ongoing
     */
    boolean pivot(double desiredAngle, double rampDownAngle, double maxPower, double minPower, double correctionTime, double correctionAngleError, Direction direction);


    //boolean moveNoIMU(double currentPosition, double targetPosition, double rampDownTargetPosition, double rampUpTargetPosition, double maxPower, double lowPower, double moveAngle, doudouble angle, double speed, boolean condition, double pivotAmount);

    //boolean balance(double highPower, double lowPower, double moveAngle, double oGain, boolean endCondition);

    /**
     * Resets the robots encoders by zeroing the current value, instead of changing the mode of the motor.
     * This should be used when you need quick, and continuous motions
     */
    void softResetEncoder();

    /**
     * Resets the robot's encoders by stopping the motors and resetting the mode of the motors
     */
    void resetEncoders();


    /**
     * Calculates how far the robot has traveled based on the drive motor encoders
     * @return the distance traveled since the last encoder reset
     */
    double getEncoderDistance();

    /**
     * Stops the drive motors and sets power to zero
     */
    void stop();

}

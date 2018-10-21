package org.firstinspires.ftc.teamcode.subsystems.drivetrain.omnidirectional;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

import java.util.List;

/**
 * Created by Sarthak on 8/1/2018.
 */
public class MecanumDrive implements IDrivetrain {

    //hardware of drivetrain
    private List<DcMotor> motors;
    private List<DcMotor> encoders;
    private IIMU imu;

    private DataLogger data;

    //How much the robot can be off the desired angle at the end of a motion
    private final double END_ANGLE_OFFSET = 5;

    private ElapsedTime pivotTime;
    private ElapsedTime distanceCorrectionTimer;

    private boolean targetReached = false;
    private boolean needsToPivot = false;

    //Variables where last encoder value when reset are stored
    private double leftVerticalLastEncoder=0;
    private double rightVerticalLastEncoder=0;
    private double horizontalLastEncoder=0;

    Telemetry telemetry;

    /**
     * Constructor for Mecanum Drivetrain
     * @param motors List of motors on drivetrain in order of Right Front, Right Back, Left Front and then Left Back
     * @param imu the inertial measurement unit or gyro sensor of the robot
     */
    public MecanumDrive(List<DcMotor> motors, IIMU imu, Telemetry telemetry){
        this.motors = motors;
        this.imu = imu;
        this.imu.initialize();
        this.telemetry = telemetry;
        pivotTime = new ElapsedTime();
        distanceCorrectionTimer = new ElapsedTime();
    }

    /**
     * Constructor for mecanum drivetrain with free-spinning odometry wheels
     * @param motors List of motors on drivetrain in order of Right Front, Right Back, Left Front and then Left Back
     * @param imu the inertial measurement unit or gyro sensor of the robot
     * @param encoders List of encoders (passed in as DcMotor) on drivetrain to calculate distances and positions
     */
    public MecanumDrive(List<DcMotor> motors, IIMU imu, Telemetry telemetry, List<DcMotor> encoders){
        this.motors = motors;
        this.imu = imu;
        this.imu.initialize();
        this.telemetry = telemetry;
        this.encoders = encoders;
        pivotTime = new ElapsedTime();
        distanceCorrectionTimer = new ElapsedTime();
    }

    /**
     * Construcor for Mecanum drivetrain that has no inertial measurement unit
     * @param motors
     * @param telemetry
     */
    public MecanumDrive(List<DcMotor> motors, Telemetry telemetry){
        this.motors = motors;
        this.imu = null;
        this.telemetry = telemetry;
        pivotTime = new ElapsedTime();
        distanceCorrectionTimer = new ElapsedTime();
    }

    /**
     * Sets power to each of the motors
     * @param rfPower the power set to the Right Front motor
     * @param rbPower the power set to the Right Back motor
     * @param lfPower the power set to the Left Front motor
     * @param lbPower the power set to the Left Back motor
     */
    private void setPowerAll(double rfPower, double rbPower, double lfPower, double lbPower){
        motors.get(0).setPower(rfPower);
        motors.get(1).setPower(rbPower);
        motors.get(2).setPower(lfPower);
        motors.get(3).setPower(lbPower);
    }

    //Takes the horizontal power, vertical power and pivoting power and determines how much power to apply to each wheel and normalizes to max power

    /**
     * Takes the horizontal power, vertical power, and pivoting power and determins how much power to apply to each wheel, and normalizes the max poer
     * @param horizontal the horizontal (x vector) power
     * @param vertical the vertical (y vector) power
     * @param pivot the pivoting power
     * @param maxPower the max power the wheels can move
     */
    public void rawSlide(double horizontal, double vertical, double pivot, double maxPower){
        //create an array with all the speeds
        double powers[] = {vertical-horizontal+pivot, vertical+horizontal+pivot, vertical+horizontal-pivot,vertical-horizontal-pivot};

        //Only adjust speeds if the robot is moving
        if(horizontal!=0 || vertical!=0){
            int max = 0;
            int counter = 0;

            //determine the maximum speed out of the four motors
            for(double element:powers){
                if(Math.abs(element)>Math.abs(powers[max])){
                    max = counter;
                }
                counter++;
            }

            //set the maximum as a variable
            double maxCalculatedPower = Math.abs(powers[max]);

            //divide all of the speeds by the max speed to make sure that
            if(maxCalculatedPower!=0){
                powers[0]=powers[0]/maxCalculatedPower*maxPower;
                powers[1]=powers[1]/maxCalculatedPower*maxPower;
                powers[2]=powers[2]/maxCalculatedPower*maxPower;
                powers[3]=powers[3]/maxCalculatedPower*maxPower;

            }
        }
        //set all the powers and display telemetry data
        this.setPowerAll(powers[0], powers[1], powers[2], powers[3]);
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Resets the drive encoder values to zero
     */
    @Override
    public void resetEncoders(){
        for(DcMotor motor:encoders){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        leftVerticalLastEncoder = 0;
        rightVerticalLastEncoder = 0;
        horizontalLastEncoder = 0;
    }
    /**
     * Resets the drive encoder values to zero
     */
    public void resetEncoders(DcMotor.RunMode endMode){
        for(DcMotor motor:encoders){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(endMode);
        }
        leftVerticalLastEncoder = 0;
        rightVerticalLastEncoder = 0;
        horizontalLastEncoder = 0;
    }

    /**
     * Moves the robot in any lateral directino (0-360 degrees) while maintaining a specific orientation
     * @param currentPosition The current position of robot in any unit eg. encoder counts, sensor distances...
     * @param targetPosition The target position of the robot in the same unit as current position
     * @param rampDownTargetPosition Position at which the robot will start ramping down
     * @param rampUpTargetPosition Position at which the robot will stop ramping up in power
     * @param maxPower The maximum power the robot will move at, ranging from 0.0 to 1.0
     * @param lowPower The lowest power the robot will move at, ranging from 0.0 to 1.0
     * @param moveAngle The angle at which the robot will move in the frame of reference of the starting position
     * @param PIDGain Three gains to control PID feedback loop for Orientation correction
     * @param endOrientationAngle The Direction the robot is facing
     * @return true once the movement is complete, false if the movement is ongoing
     */
    @Override
    public boolean move(double currentPosition, double targetPosition, double rampDownTargetPosition, double rampUpTargetPosition, double rampDownEnd, double maxPower, double lowPower, double moveAngle, double[] PIDGain, double endOrientationAngle, double allowableDistanceError, double correctionTime) {

        //get the difference between the target position and the current position
        double positionDifference = targetPosition - currentPosition;

        //If the robot is within the window of error for distance movements, stop motors
        if(Math.abs(positionDifference)<=allowableDistanceError){
            this.stop();

            if(!targetReached){
                targetReached = true;
                distanceCorrectionTimer.reset();
                return true;
            }

        }else{

            //Calculate the difference between the target position and the position the robot begins to ramp down
            double rampDownDifference = targetPosition - rampDownTargetPosition;
            double rampDownEndDifference = targetPosition - rampDownEnd;

            //Calculate robot power
            double power;
            if(rampDownEndDifference>=Math.abs(positionDifference)) {
                power = lowPower;
            }else if(rampDownDifference>Math.abs(positionDifference)){ //Find the speed based on the speed of the robot's slope and distance traveled
                power = (Math.abs(positionDifference)-rampDownDifference)*((maxPower-lowPower)/(rampDownDifference-rampDownEndDifference))+maxPower;
            }else{
                power = maxPower;
            }

            //Calculate orientation corrections
            //Get the current imu angle
            double currentAngle = imu.getZAngle(endOrientationAngle);
            //Calculate angle at which to slide at
            moveAngle = moveAngle - currentAngle;
            //Correct for IMU discontinuity
            if(moveAngle <= -180){
                moveAngle+=360;
            }
            if(positionDifference<0){
                moveAngle+=180;
            }
            //Calculate the x vector for movement
            double horizontal = Utilities.round2D(calculateX(moveAngle, power));
            //Calculate the y vector for movement
            double vertical = Utilities.round2D(calculateY(moveAngle, power));
            //Calculate the correction to maintain the robot's orientation
            double pivotCorrection = ((currentAngle - endOrientationAngle) * PIDGain[0]);
            //Calculate individual motor speeds
            rawSlide(horizontal, vertical, pivotCorrection, power);
        }
        //Check if the robot has corrected for the desired amount of time
        if(targetReached&&distanceCorrectionTimer.milliseconds()>=correctionTime){
            this.stop();
            targetReached = false;
            return false;
        }else{
            return true;
        }
    }

    /**
     * Pivots the robot to a desired angle, while using a proportional control loop to maintain the robot's drive speed
     * @param desiredAngle The angle to which to pivot to
     * @param rampDownAngle The angle at which to start slowing down
     * @param maxPower The max power to pivot at, ranging from 0.0 to 1.0
     * @param minPower The min power to pivot at, ranging from 0.0 to 1.0
     * @param correctionAngleError
     * @param correctionTime The amount of time to spend correcting to stay within the desired range
     * @param direction
     * @return true if the action has been completed, false if the robot is still pivoting
     */
    @Override
    public boolean pivot(double desiredAngle, double rampDownAngle, double maxPower, double minPower, double correctionAngleError, double correctionTime, Direction direction) {
        double currentAngle = imu.getZAngle(desiredAngle);
        double angleDifference = desiredAngle-currentAngle;
        double rampDownDifference = desiredAngle - rampDownAngle;
        double power;

        //calculate power
        if(Math.abs(angleDifference)>Math.abs(rampDownDifference)){
            power = maxPower;
        }else{
            power = (maxPower-minPower)/(Math.abs(rampDownDifference)) * Math.abs(angleDifference) + minPower;
        }
        //turn clockwise or counterclockwise depending on which side of desired angle current angle is
        if(direction== Direction.FASTEST||targetReached){
            if(angleDifference>0){
                this.setPowerAll(-power, -power, power, power);
            }else{
                this.setPowerAll(power, power, -power, -power);
            }
        }else if(direction == Direction.CLOCKWISE){
            this.setPowerAll(-power, -power, power, power);
        }else{
            this.setPowerAll(power, power, -power, -power);
        }


        //determine if the pivoting angle is in the desired range
        if(Math.abs(angleDifference)<correctionAngleError&&!targetReached){
            pivotTime.reset();
            targetReached = true;
        }
        if(targetReached && pivotTime.milliseconds()>=correctionTime){
            targetReached = false;
            this.stop();
            return false;
        }else{
            return true;
        }
    }

    /**
     * Resets the encoders by setting the current encoder position as position 0
     */
    @Override
    public void softResetEncoder(){
        leftVerticalLastEncoder = encoders.get(0).getCurrentPosition();
        rightVerticalLastEncoder = encoders.get(1).getCurrentPosition();
        horizontalLastEncoder = encoders.get(2).getCurrentPosition();
    }


    /**
     * Get the individual motor encoder positions
     * @return the motor encoders in a double array, in the order of right front, right back, left front, left back
     */
    private double[] getEncoderPositions(){
        double[] encoders = {this.encoders.get(0).getCurrentPosition()-leftVerticalLastEncoder, this.encoders.get(1).getCurrentPosition()-rightVerticalLastEncoder,
                this.encoders.get(2).getCurrentPosition()-horizontalLastEncoder};
        return encoders;
    }

    /**
     * Returns the current distance traveled/encoder position by averaging the position of each drive motor
     * @return distance traveled as a double
     */
    public double getEncoderDistance(){

        //Get Current Positions
        double []encoders = this.getEncoderPositions();

        double vlPos = encoders[0];
        double vrPos = encoders[1];
        double hPos = encoders[2];

        //Average the Vertical Wheels
        double y = ((vlPos + vrPos) / 2);
        double x = hPos;

        //Calculate distance
        double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));

        return distance;
    }

    /**
     * Turns off the drivetrain and stops all threads related to the driveetrain
     */
    @Override
    public void stop() {
        setPowerAll(0, 0, 0, 0);
    }

}

package org.firstinspires.ftc.teamcode.GandalfCode;

//import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.Locale;

/**
 * Created by Raashi on 1/7/2018.
 */

public class MecanumDrive {

    private DcMotor rf;
    private DcMotor rb;
    private DcMotor lf;
    private DcMotor lb;
    private ModernRoboticsI2cCompassSensor compass;
    private ModernRoboticsAnalogOpticalDistanceSensor light;
    private ModernRoboticsI2cRangeSensor ultrasonic;
    private ModernRoboticsI2cGyro gyro;
    private Telemetry telemetry;
    //private AHRS navx_device;
    private ElapsedTime pivotTime;
    private ElapsedTime moveTime;
    public boolean usingIMU = true;
    public static int sensor = 2;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    //create variables
    private double currentAngle;
    private double newSpeed;
    private double angleDifference;
    private double wallReading;
    private double vertical;
    private double horizontal;
    private double IMUAngle;
    private double angleChange;
    boolean notVisible = true;
    private double rfLastEncoder;
    private double rbLastEncoder;
    private double lfLastEncoder;
    private double lbLastEncoder;
    private ElapsedTime timer = new ElapsedTime();



    /*
    Inputs:         The four motors
                    IMU Sensor
                    Telemetry
    Outputs:        None
    Description:    Constructor for Mecanum drive object
     */
    public MecanumDrive(DcMotor rf, DcMotor rb, DcMotor lf, DcMotor lb, ModernRoboticsI2cGyro gyro, Telemetry telemetry) {
        //Initialize motors, and IMU sensor
        this.rf = rf;
        this.rb = rb;
        this.lf = lf;
        this.lb = lb;
        this.gyro = gyro;
        this.telemetry = telemetry;
    }
    /*public MecanumDrive(DcMotor rf, DcMotor rb, DcMotor lf, DcMotor lb, ModernRoboticsI2cGyro gyro, AHRS navx_device, Telemetry telemetry) {
        //Initialize motors, and IMU sensor
        this.navx_device = navx_device;
        this.rf = rf;
        this.rb = rb;
        this.lf = lf;
        this.lb = lb;
//        this.imu = imu;
        this.gyro = gyro;
        this.telemetry = telemetry;
    }*/
    /*
    Inputs:         Angle: Clockwise is positive; it can go negative
                    Gain: How much the robot will change the power per degree as it approaches the desired angle (between 0 and 1)
                    Time: How long the robot will take to get to the desired angle
                    Max Speed: Double between 0 and 1
                    Min Speed: Double between 0 and 1
                    Active: Boolean expression of whether the op mode is active or not
    Outputs:        Return of Active - 0 or 1 - which determines whether the op mode is done or not
    Description:    This method pivots the robot to a desired angle. Because of the gain, the robot speed changes.
                    Max speed and speed limit the range of speed. The power used is the gain times the difference
                    of the current angle vs the desired (0-doesn't move, 1- moves at full force)
     */

    public int pivotToAngleIMU(double desiredAngle, double gain, boolean endCase, double maxSpeed, double minSpeed) {
        //set speeds until time is up
        if (endCase) {
            //measure the gyro sensor
            //get gyro sensor value
//            if(sensor == 1){
//                Orientation o = imu.getAngularOrientation();
//                currentAngle = -o.firstAngle; //Convert negative calculation to positive
            if (sensor == 0){
                currentAngle = gyro.getHeading();
            }else if(sensor == 2){
                //currentAngle = navx_device.getYaw();
            }

            double difference = desiredAngle - currentAngle;
            if(difference>180){
                difference-=360;
            }else if(difference<-180){
                difference+=360;
            }

            double magSpeed = Math.abs(difference * gain);
            telemetry.addData("Mag speed", magSpeed);
            if(magSpeed > maxSpeed){
                magSpeed = maxSpeed;
            }
            if(magSpeed<minSpeed){
                magSpeed = minSpeed;
            }
            if(difference<0){
                rf.setPower(magSpeed);
                rb.setPower(magSpeed);
                lf.setPower(-magSpeed);
                lb.setPower(-magSpeed);
            }else{
                rf.setPower(-magSpeed);
                rb.setPower(-magSpeed);
                lf.setPower(magSpeed);
                lb.setPower(magSpeed);
            }
            //send back data about what the robot is doing


            telemetry.addData("Angle", currentAngle);
            telemetry.addData("Desired Angle", desiredAngle);
            telemetry.addData("angle difference", currentAngle-desiredAngle);
            return 1;
        } else {
            return 0;
        }
    }

    /*
    Inputs:         Speeds: Four doubles from -1 to 1 representing the speed of each motor (RF, RB, LF, LB)
                    Desired Light: The amount of reflected light the robot needs to follow (0 to 1)
                    Gain: How much the robot will change the power per .01 light intensity off the desired light (0 to 1)
                    leftOrRight: String "left" or "right" determining whether you follow the left or right side of the line
                    endCase: Is 0 when the robot will stop line following
    Outputs:        Return of endCase: 0 makes the robot stop following the line
    Description:    This method follows a line
     */


    /*
    Inputs:         Speeds: Four doubles from -1 to 1 representing the speed of each motor (RF, RB, LF, LB)
                    Light: How much light you would like your robot to see (int between 0 - dark and 256 - white light)
                    Sentinel which controls the execution of the loop (0 or 1)
    Outputs:        Return 0 makes the robot stop and 1 keeps it going
    Description:    This method will move the robot until a certain light value has been detected
     */



    /*
    Inputs:         Four doubles from -1 to 1 representing the speed of each motor (RF, RB, LF, LB)
    Outputs:        none
    Description:    Sets power to all of the motors
     */
    public void setPowerAll(double rfSpeed, double rbSpeed, double lfSpeed, double lbSpeed) {
        rf.setPower(rfSpeed);
        rb.setPower(rbSpeed);
        lf.setPower(lfSpeed);
        lb.setPower(lbSpeed);
    }





    /*
    Pivot slide slides the robot at an angle while allowing it to pivot a certain amount
    The parameters are angle which is a double in the frame of reference of the robot
    speed which is a double between -1 and 1
    the condition is a boolean value which is when the program will stop and return 0
    the pivotAmount is a double between -1 and 1 which is the speed at which it will pivot
    Difference between this method and slideAngle is you can set the power you are pivoting at
    for smooth turns
    Method mostly for teleop
     */

    public int pivotSlide(double angle, double speed, boolean condition, double pivotAmount) {

        //return the new X and Y values using the angle needed and the speed the robot was
        //traveling at
        horizontal = round2D(calculateX(angle, speed));
        vertical = round2D(calculateY(angle, speed));

        //determine the powers using the new X and Y values and the other joystick to pivot
        if (condition) {
            rawSlide(horizontal, vertical, pivotAmount, speed);
            return 1;
        } else {
            return 0;
        }

    }
    //Convert IMU reading to value between -infinity and infinity
    double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.parseDouble(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    public int slideAngleIMU(double slideDirection, double speed, boolean condition, double orientation, double oGain) {

        // get the horizonal and vertical components of the robot speeds
        // the horizonal and vertical are used to set the power of the motor
        horizontal = round2D(calculateX(slideDirection, speed));
        vertical = round2D(calculateY(slideDirection, speed));

        if (sensor == 0){
            currentAngle = gyro.getHeading();

        }else if(sensor == 2){
            //IMUAngle = navx_device.getYaw();
        }
        if(currentAngle>orientation+180){
            IMUAngle = currentAngle-360;
        }else if(currentAngle<orientation-180){
            IMUAngle = currentAngle+360;
        }




        // What is the range of the gyroAngle?
        double pivotCorrection = -((orientation - IMUAngle) * oGain);

        //determine the powers using the new X and Y values and the other joystick to pivot
        if (condition) {
            this.rawSlide(horizontal, vertical, pivotCorrection, speed);
            return 1;
        } else {
            return 0;
        }
    }
    public void slideAngleFrame(double offset, double moveAngle, double orientationAngle, double highPower, double lowPower, double powerPercent, double powerGain, double oGain) {
        //double currentAngle = navx_device.getYaw()-offset;
        boolean fixAngle = true;
        while(fixAngle){
            if(currentAngle>orientationAngle+180){
                currentAngle = currentAngle-360;
            }else if(currentAngle<orientationAngle-180){
                currentAngle = currentAngle+360;
            }else{
                fixAngle = false;
            }
        }
        moveAngle = moveAngle - currentAngle;
        telemetry.addData("move angle", moveAngle);
        double power = Math.max(Math.min(powerPercent*powerGain, highPower), lowPower);
        double horizontal = round2D(calculateX(moveAngle, power));
        double vertical = round2D(calculateY(moveAngle, power));
        double pivotCorrection = -((orientationAngle - currentAngle) * oGain);
        rawSlide(horizontal, vertical, pivotCorrection, power);
    }

    public void rawSlide(double horizontal, double vertical, double pivot, double speed){
        //create an array with all teh speeds
        double speeds[] = {vertical-horizontal+pivot, vertical+horizontal+pivot, vertical+horizontal-pivot,vertical-horizontal-pivot};

        //Only adjust speeds if the robot is moving
        if(horizontal!=0 || vertical!=0){
            int max = 0;
            int counter = 0;

            //determine the maximum speed out of the four motors
            for(double element:speeds){
                if(Math.abs(element)>Math.abs(speeds[max])){
                    max = counter;
                }
                counter++;
            }

            //set the maximum as a variable
            double maxSpeed = Math.abs(speeds[max]);

            //divide all of the speeds by the max speed to make sure that
            if(maxSpeed!=0){
                speeds[0]=speeds[0]/maxSpeed*speed;
                speeds[1]=speeds[1]/maxSpeed*speed;
                speeds[2]=speeds[2]/maxSpeed*speed;
                speeds[3]=speeds[3]/maxSpeed*speed;
            }
        }

        //set all the powers and display telemetry data
        this.setPowerAll(speeds[0], speeds[1], speeds[2], speeds[3]);
        telemetry.addData("rf", speeds[0]);
        telemetry.addData("rb", speeds[1]);
        telemetry.addData("lf", speeds[2]);
        telemetry.addData("lb", speeds[3]);;
    }

    // returns how far the joystick is pushed in distance from the center
    // inputs are the X value and Y joystick value
    public double returnRadius(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    // takes the X and Y value of the joystick and determines
    // the angle that the joystick is at relative to the center of the joystcik
    // returns in degrees
    public double joystickToAngle(double x, double y) {
        return Math.atan2(x, y) * (180 / Math.PI);
    }

    //returns X vector value using angle and speed
    public double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    //returns the Y vector value using angle and speed
    public double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    //rounds the input to 2 decimal places
    public double round2D(double input) {
        input *= 100;
        input = Math.round(input);
        return input / 100;
    }

    public void resetEncoders(){
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void softResetEncoder(){
        rfLastEncoder = rf.getCurrentPosition();
        lfLastEncoder = lf.getCurrentPosition();
        rbLastEncoder = rb.getCurrentPosition();
        lbLastEncoder = lb.getCurrentPosition();
        
    }

    public double[] getEncoderPositions(){
        double[] encoders = {rf.getCurrentPosition()-rfLastEncoder, rb.getCurrentPosition()-rbLastEncoder, lf.getCurrentPosition()-lfLastEncoder, lb.getCurrentPosition()-lbLastEncoder};
        return encoders;
    }
    public double averageEncoders(){
        double []encoders = this.getEncoderPositions();
        double encoderA = (encoders[2] + encoders[1]) / 2;
        double encoderB = (encoders[0] + encoders[3]) / 2;
        double encoderAverage = (Math.abs(encoderA) + Math.abs(encoderB)) / 2;
        return encoderAverage;
    }
    public double[] getVuforiaPosition(VectorF beacons){
        double[] vuforiaPosXYZ = {beacons.get(0), beacons.get(1), beacons.get(2)};
        return vuforiaPosXYZ;
    }
    public double getVuforiaAngle(double xDif, double zDif){
        return Math.toDegrees((Math.atan2(zDif, xDif) - 180)) % 360;
    }

    public int vuforiaMoveToPosition(VuforiaTrackableDefaultListener target, double desiredX, double desiredZ){
        OpenGLMatrix pose = target.getPose();
        if(pose!= null){
            VectorF translation = pose.getTranslation();
            double x = translation.get(0);
            double z = translation.get(2);
            double xDif = x - desiredX;
            double zDif = z-desiredZ;
            double vuforiaAngle = -getVuforiaAngle(xDif, zDif);
            if (x>desiredX || z>desiredZ){
                telemetry.addData("Sliding", "Angle "+vuforiaAngle);
                if(pose!=null){
                    translation = pose.getTranslation();
                    x = translation.get(0);
                    z = translation.get(2);
                }
                telemetry.update();
                return 1;
            }
            else {
                telemetry.addData("Set Power", 0);
                telemetry.update();
                return 1;
            }
        }else{
            timer.reset();
            while(timer.seconds() < 1){
                pose = target.getPose();
            }
            if(pose==null){
                return 0;
            }else{
                return 1;
            }
        }
    }
}

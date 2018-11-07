package org.firstinspires.ftc.teamcode.autonomous;

import android.media.AudioManager;
import android.media.SoundPool;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.omnidirectional.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

import java.util.ArrayList;

/**
 * Created by Sarthak on 10/26/2018.
 */
@Autonomous(name = "Double Sampling Autonomous", group = "Autonomous")
public class RoverRuckusDoubleSamplingAutonomous extends LinearOpMode {

    IDrivetrain drive;
    DcMotor right_front, right_back, left_front, left_back;
    DcMotor verticalLeft, verticalRight, horizontal, horizontal2;
    ArrayList motors, encoders;

    IIMU imu;
    BNO055IMU boschIMU;

    ElapsedTime timer = new ElapsedTime();

    //Declare OpMode timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime centertimer = new ElapsedTime();
    boolean notReset = false;

    //Define possible mineral locations in enum
    enum location {
        LEFT, CENTER, RIGHT, UNKNOWN
    };

    //Create location object to store the mineral location data
    location mineralLocation;

    //Create detector to be used for the gold mineral
    private GoldDetector genericDetector = null;

    //define constants for drive movement parameters
    final double DEFAULT_MAX_POWER = .75;
    final double DEFAULT_MIN_POWER = .25;
    final double DEFAULT_MIN_POWER_PIVOT = .15;

    final double DEFAULT_ERROR_DISTANCE = 10;

    final double[] DEFAULT_PID = {.025};
    final double[] DEFAULT_PID_STRAFE = {.03};
    final double COUNTS_PER_INCH = 307.699557;

    //Position variables
    double vrPos = 0, vlPos = 0, hPos = 0;
    double x = 0, y = 0, angle = 0;
    final double alpha = 53.13;

    double prevRight = 0, prevLeft = 0, prevHorizontal = 0;
    double length = 13.25 * COUNTS_PER_INCH;

    double changeInPosition = 0, changeInAngle = 0;
    double changeInX = 0, changeInY = 0;

    SoundPool sound;
    int beepID;

    @Override
    public void runOpMode() throws InterruptedException {
        sound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
        beepID = sound.load(hardwareMap.appContext, R.raw.supermariobros, 1);

        //Init motor hardware map and behaviors
        setMotorBehaviors();

        genericDetector = new GoldDetector();
        genericDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        centertimer = new ElapsedTime();

        telemetry.addData("Status", "Vision Initialized.");

        //Initialize IMU
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.initialize();
        imu.setOffset(0);
        telemetry.addData("Status", "IMU Instantiated");
        telemetry.update();

        //Setup Drivetrain Subsystem
        drive = new MecanumDrive(motors, imu, telemetry, encoders);
        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();
        genericDetector.enable();
        genericDetector.disable();
        runtime.reset();

        /**
         * *****************************************************************************************
         * *****************************************************************************************
         * *******************************OPMODE RUNS HERE******************************************
         * *****************************************************************************************
         * *****************************************************************************************
         */

        globalCoordinatePositionUpdate();

        //Scan for mineral

        globalCoordinatePositionUpdate();

        //Determine mineral location
        mineralLocation = location.LEFT;
        globalCoordinatePositionUpdate();

        //Begin Sampling
        while(goToPosition(-10*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER)
                && opModeIsActive()){
            globalCoordinatePositionUpdate();
            telemetry.addData("Moving to Position", "(-10, 0)");
            telemetry.update();
        }
        drive.stop();
        globalCoordinatePositionUpdate();

        waitMilliseconds(500, runtime);

        switch (mineralLocation){
            case CENTER:
                globalCoordinatePositionUpdate();

                //Knock Mineral
                while(goToPosition(-21*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, -90, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-21, 0)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();
                genericDetector.disable();

                //Intake mineral
                waitMilliseconds(1000, runtime);

                //Go to alliance depot to deposit the team marker
                globalCoordinatePositionUpdate();
                while (goToPosition(-50*COUNTS_PER_INCH, -3*COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(0, 50)");
                    telemetry.update();
                }
                drive.stop();

                //Deposit team marker
                waitMilliseconds(1000, timer);

                break;

            case LEFT:
                globalCoordinatePositionUpdate();

                //Knock Mineral
                while(goToPosition(-21*COUNTS_PER_INCH, -18*COUNTS_PER_INCH, -90, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-21, 0)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();
                genericDetector.disable();

                //Intake mineral
                waitMilliseconds(1000, runtime);

                //Go to alliance depot
                while(goToPosition(-45*COUNTS_PER_INCH, -17*COUNTS_PER_INCH, 45, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-21, 0)");
                    telemetry.update();
                }
                while(goToPosition(-54*COUNTS_PER_INCH, -9*COUNTS_PER_INCH, 45, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-21, 0)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                //Deposit Team Marker
                waitMilliseconds(1000, runtime);

                break;

            case RIGHT:
                globalCoordinatePositionUpdate();

                //Knock Mineral
                while(goToPosition(-21*COUNTS_PER_INCH, 15*COUNTS_PER_INCH, -90, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-21, 0)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();
                genericDetector.disable();

                //Intake mineral
                waitMilliseconds(1000, runtime);

                //Go to alliance depot
                while(goToPosition(-48*COUNTS_PER_INCH, 14*COUNTS_PER_INCH, -45, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-21, 0)");
                    telemetry.update();
                }
                while(goToPosition(-56*COUNTS_PER_INCH, 6*COUNTS_PER_INCH, -45, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-21, 0)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                //Deposit team marker
                waitMilliseconds(1000, runtime);
        }

        //Move to second sampling area
        while(goToPosition(-41*COUNTS_PER_INCH, 24*COUNTS_PER_INCH, -45, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER)
                && opModeIsActive()){
            globalCoordinatePositionUpdate();
            telemetry.addData("Moving to Position", "(-21, 0)");
            telemetry.update();
        }
        while(goToPosition(-26*COUNTS_PER_INCH, 41*COUNTS_PER_INCH, -45, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER)
                && opModeIsActive()){
            globalCoordinatePositionUpdate();
            telemetry.addData("Moving to Position", "(-21, 0)");
            telemetry.update();
        }
        drive.stop();
        globalCoordinatePositionUpdate();

        if(mineralLocation == location.CENTER || mineralLocation == location.RIGHT) {
            while (goToPosition(15 * COUNTS_PER_INCH, 38 * COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER)
                    && opModeIsActive()) {
                globalCoordinatePositionUpdate();
                telemetry.addData("Moving to Position", "(-21, 0)");
                telemetry.update();
            }
            drive.stop();
            globalCoordinatePositionUpdate();
        }

        //Knock off second gold mineral
        switch (mineralLocation){
            case CENTER:
                while(goToPosition(15*COUNTS_PER_INCH, 56*COUNTS_PER_INCH, 0, 0.4, DEFAULT_MIN_POWER)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-21, 0)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                break;

            case LEFT:
                while(goToPosition(-1*COUNTS_PER_INCH, 40*COUNTS_PER_INCH, 0, 0.4, DEFAULT_MIN_POWER)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-21, 0)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                while(goToPosition(-1*COUNTS_PER_INCH, 50*COUNTS_PER_INCH, 0, 0.4, DEFAULT_MIN_POWER)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-21, 0)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                break;

            case RIGHT:
                while(goToPosition(33*COUNTS_PER_INCH, 52*COUNTS_PER_INCH, 0, 0.4, DEFAULT_MIN_POWER)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-21, 0)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                break;
        }

        sound.play(beepID, 1, 1, 1, 0, 1);
        while (opModeIsActive()){
            drive.stop();
            globalCoordinatePositionUpdate();
            telemetry.addData("Status", "Program Finished");
            telemetry.addData("X Position", x/COUNTS_PER_INCH);
            telemetry.addData("Y Position", y/COUNTS_PER_INCH);
            telemetry.update();
        }

    }

    /**
     * Pivot the robot so that the robot can find the gold mineral. The robot wants to get the gold mineral in the camera frame
     * @param scanRadius degrees to pivot, to scan for the mineral in each direction
     * @return true if the block was found, false if the block was not found
     */
    private boolean scanBlock(double scanRadius){
        //Determine if the block is already in the frame
        boolean blockFound = genericDetector.isFound();

        //Pivot to -scanRadius degrees. If the block is in the frame as the robot pivots, the pivot will stop
        while(drive.pivot(-scanRadius, -30, 0.2, 0.15, 500, 1, Direction.FASTEST) && !blockFound &&opModeIsActive()){
            if(genericDetector.isFound()){ //Record whether the gold mineral is in the frame
                blockFound = true;
            }
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.addData("Block Detected", genericDetector.isFound());
            telemetry.update();
        }

        drive.stop();

        //If the block wasn't in the frame after the first pivot, pivot to scanRadius degrees
        if(!blockFound){
            while(drive.pivot(scanRadius, 0, 0.2, 0.15, 500, 1, Direction.FASTEST) && !blockFound &&opModeIsActive()){
                if(genericDetector.isFound()){
                    blockFound = true;
                }
                telemetry.addData("IMU Angle", imu.getZAngle());
                telemetry.addData("Block Detected", genericDetector.isFound());
                telemetry.update();
            }
        }

        //Return whether the block was found after the scan
        return blockFound;
    }

    /**
     * Stop all actions for a specified amount of time (in milliseconds)
     * @param milliseconds amount of time to wait
     * @param timer ElapsedTimer object to keep track of the time
     */
    private void waitMilliseconds(double milliseconds, ElapsedTime timer){
        //Reset the timer
        timer.reset();
        //Wait until the time inputted has fully elapsed
        while(opModeIsActive() && timer.milliseconds() < milliseconds){
            globalCoordinatePositionUpdate();
        }
    }

    /**
     * Setup the hardware map and the motor modes, zero power behaviors, and direction
     */
    private void setMotorBehaviors(){
        //Hardware Map
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");

        verticalLeft = hardwareMap.dcMotor.get("right_front");
        verticalRight = hardwareMap.dcMotor.get("right_back");
        horizontal = hardwareMap.dcMotor.get("left_front");
        horizontal2 = hardwareMap.dcMotor.get("left_back");

        //Set motor behaviors
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoders = new ArrayList<>();
        encoders.add(verticalLeft);
        encoders.add(verticalRight);
        encoders.add(horizontal);
        encoders.add(horizontal2);

        //Set Zero Power Behavior
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        //Put drive motors in ArrayList to pass into drivetrain object
        motors = new ArrayList<>();
        motors.add(right_front);
        motors.add(right_back);
        motors.add(left_front);
        motors.add(left_back);

        //Status update
        telemetry.addData("Status", "Motor Hardware Initialized");
        telemetry.update();
    }

    private boolean goToPosition(double targetX, double targetY, double targetOrientation, double maxPower, double minPower){

        double xDistance = targetX - x;
        double yDistance = targetY - y;

        double orientationDifference = targetOrientation - imu.getZAngle();

        double distance = distanceFormula(xDistance, yDistance);
        double power = (distance/COUNTS_PER_INCH) * DEFAULT_PID[0];

        if (Math.abs(power) > maxPower){
            power = maxPower;
        }else if(Math.abs(power) < minPower){
            power = minPower;
        }

        double moveAngle = 0;
        moveAngle = Math.toDegrees(Math.atan(xDistance/yDistance));
        if((xDistance < 0 && yDistance < 0) || (xDistance > 0 && yDistance < 0)){
            moveAngle += 180;
        }
        moveAngle = (moveAngle % 360) - angle;

        if(!(Math.abs(yDistance) < 0.3 * COUNTS_PER_INCH && Math.abs(xDistance) < 0.3 * COUNTS_PER_INCH
                && Math.abs(orientationDifference) < 2)){
            drive.move(0, distance, distance, 0, distance, power, power,
                    moveAngle, DEFAULT_PID, targetOrientation, DEFAULT_ERROR_DISTANCE, 500);
            telemetry.addData("Distance", distance/COUNTS_PER_INCH);
            telemetry.addData("X Distance", xDistance/COUNTS_PER_INCH);
            telemetry.addData("Y Distance", yDistance/COUNTS_PER_INCH);
            telemetry.addData("Move Angle", moveAngle);

            if((Math.abs(yDistance) < 0.5 * COUNTS_PER_INCH && Math.abs(xDistance) < 0.5 * COUNTS_PER_INCH
                    && Math.abs(orientationDifference) < 2) && !notReset){
                centertimer.reset();
                notReset = true;
            }

            if((Math.abs(yDistance) < 0.5 * COUNTS_PER_INCH && Math.abs(xDistance) < 0.5 * COUNTS_PER_INCH
                    && Math.abs(orientationDifference) < 2) && centertimer.milliseconds() > 2000 && notReset){
                return false;
            }

            return true;
        }else{
            notReset = false;
            return false;
        }

    }

    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        vlPos = verticalLeft.getCurrentPosition();
        vrPos = verticalRight.getCurrentPosition();
        hPos = horizontal.getCurrentPosition();

        double leftChange = vlPos - prevLeft;
        double rightChange = vrPos - prevRight;
        double horizontalChange = hPos - prevHorizontal;

        //Calculate Angle
        changeInAngle = (leftChange - rightChange) / (length);
        angle = ((angle + changeInAngle));

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange + (((leftChange-rightChange)/2) * Math.sin(alpha));
        x = x + (p*Math.sin(angle) + n*Math.cos(angle));
        y = y + -(p*Math.cos(angle) - n*Math.sin(angle));

        prevLeft = vlPos;
        prevRight = vrPos;
        prevHorizontal = hPos;

        telemetry.addData("X Position", x / COUNTS_PER_INCH);
        telemetry.addData("Y Position", y / COUNTS_PER_INCH);
    }

    public double distanceFormula(double x, double y){
        double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
        return distance;
    }

}

package org.firstinspires.ftc.teamcode.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.omnidirectional.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.sampling.GoldMineralDetector;

import java.util.ArrayList;

/**
 * Created by Sarthak on 10/29/2018.
 */
@Autonomous(name = "Secondary Autonomous", group = "Autonomous")
public class RoverRuckusSecondaryAutonomousProgram extends LinearOpMode {
    IDrivetrain drive;
    DcMotor right_front, right_back, left_front, left_back;
    DcMotor verticalLeft, verticalRight, horizontal, horizontal2;
    ArrayList motors, encoders;

    IIMU imu;
    BNO055IMU boschIMU;

    ElapsedTime timer = new ElapsedTime();

    //Declare OpMode timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime centertimer;

    //Define possible mineral locations in enum
    enum location {
        LEFT, CENTER, RIGHT, UNKNOWN
    };

    //Create location object to store the mineral location data
    location mineralLocation;

    //Create detector to be used for the gold mineral
    private GoldMineralDetector genericDetector = null;

    //define constants for drive movement parameters
    final double DEFAULT_MAX_POWER = .75;
    final double DEFAULT_MIN_POWER = .25;
    final double DEFAULT_MIN_POWER_PIVOT = .15;

    final double DEFAULT_ERROR_DISTANCE = 10;

    final double[] DEFAULT_PID = {.05};
    final double[] DEFAULT_PID_STRAFE = {.03};
    final double COUNTS_PER_INCH = 307.699557;

    //Position variables
    double vrPos = 0, vlPos = 0, hPos = 0;
    double x = 0, y = 0, angle = 0;

    double prevRight = 0, prevLeft = 0, prevHorizontal = 0;
    double length = 13.25 * COUNTS_PER_INCH;

    double changeInPosition = 0, changeInAngle = 0;
    double changeInX = 0, changeInY = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Init motor hardware map and behaviors
        setMotorBehaviors();

        genericDetector = new GoldMineralDetector();
        genericDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        genericDetector.colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW);
        genericDetector.enable();

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
        genericDetector.disable();
        runtime.reset();

        /**
         * *****************************************************************************************
         * *****************************************************************************************
         * *******************************OPMODE RUNS HERE******************************************
         * *****************************************************************************************
         * *****************************************************************************************
         */

        //Pivot back to calibration point
        while(opModeIsActive() && runtime.milliseconds() < 1000){
            drive.pivot(0, 0, 0.25, 0.2, 500, 5, Direction.FASTEST);
        }
        drive.stop();

        //Scan for mineral

        //Determine mineral location
        mineralLocation = location.RIGHT;

        //Knock off the mineral based on its location
        switch (mineralLocation){
            case CENTER:
                drive.softResetEncoder();
                //Knock gold mineral off
                while(drive.move(drive.getEncoderDistance(), 30*COUNTS_PER_INCH, 15*COUNTS_PER_INCH, 0,
                        30*COUNTS_PER_INCH, 0.35, 0.15, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                drive.stop();

                //Intake mineral
                waitMilliseconds(1000, timer);

                //Go back to starting position
                drive.softResetEncoder();
                while(drive.move(drive.getEncoderDistance(), 34*COUNTS_PER_INCH, 17*COUNTS_PER_INCH, 0,
                        34*COUNTS_PER_INCH, 0.35, 0.15, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                drive.stop();

                //Pivot back to face perimeter
                while(opModeIsActive() && runtime.milliseconds() < 2500){
                    drive.pivot(-45, -22.5, 0.25, 0.2, 500, 5, Direction.FASTEST);
                }
                drive.stop();

                //Drive to perimeter wall
                drive.softResetEncoder();
                while(drive.move(drive.getEncoderDistance(), 46*COUNTS_PER_INCH, 23*COUNTS_PER_INCH, 0,
                        46*COUNTS_PER_INCH, 0.35, 0.15, -45, DEFAULT_PID, -45, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                drive.stop();

                //Pivot to face alliance depot
                runtime.reset();
                while(opModeIsActive() && runtime.milliseconds() < 2000){
                    drive.pivot(-135, -90, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 500, 5, Direction.FASTEST);
                }
                drive.stop();

                //Drive to alliance depot to deposit team marker

                break;

            case LEFT:
                //Knock gold mineral off
                drive.softResetEncoder();
                while(drive.move(drive.getEncoderDistance(), 25*COUNTS_PER_INCH, 15*COUNTS_PER_INCH, 0,
                        25*COUNTS_PER_INCH, 0.35, 0.15, -45, DEFAULT_PID_STRAFE, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                drive.stop();

                //Pivot to intake mineral
                runtime.reset();
                while(opModeIsActive() && runtime.milliseconds() < 1000){
                    drive.pivot(10, 22.5, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 500, 5,
                            Direction.FASTEST);
                    telemetry.addData("Status", "Drive Pivoting");
                    telemetry.update();
                }
                drive.stop();

                //Intake Gold Mineral
                waitMilliseconds(1000, runtime);

                //Pivot to face perimeter
                runtime.reset();
                while(opModeIsActive() && runtime.milliseconds() < 2000){
                    drive.pivot(225, 180, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 500, 5,
                            Direction.FASTEST);
                    telemetry.addData("Status", "Drive Pivoting");
                    telemetry.update();
                }
                drive.stop();

                //Drive towards perimeter wall
                drive.softResetEncoder();
                while(drive.move(drive.getEncoderDistance(), 20*COUNTS_PER_INCH, 10*COUNTS_PER_INCH, 0,
                        20*COUNTS_PER_INCH, 0.35, 0.15, -45, DEFAULT_PID_STRAFE, 225, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                drive.stop();

                break;

            case RIGHT:
                //Knock gold mineral off
                drive.softResetEncoder();
                while(drive.move(drive.getEncoderDistance(), 25*COUNTS_PER_INCH, 15*COUNTS_PER_INCH, 0,
                        25*COUNTS_PER_INCH, 0.35, 0.15, 45, DEFAULT_PID_STRAFE, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                drive.stop();

                //Pivot to intake mineral
                runtime.reset();
                while(opModeIsActive() && runtime.milliseconds() < 1000){
                    drive.pivot(-10, 22.5, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 500, 5,
                            Direction.FASTEST);
                    telemetry.addData("Status", "Drive Pivoting");
                    telemetry.update();
                }
                drive.stop();

                waitMilliseconds(1000, runtime);

                //Pivot to original position
                runtime.reset();
                while(opModeIsActive() && runtime.milliseconds() < 1000){
                    drive.pivot(0, 22.5, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 500, 5,
                            Direction.FASTEST);
                    telemetry.addData("Status", "Drive Pivoting");
                    telemetry.update();
                }
                drive.stop();

                //Move back to original position
                drive.softResetEncoder();
                while(drive.move(drive.getEncoderDistance(), 30*COUNTS_PER_INCH, 30*COUNTS_PER_INCH, 0,
                        30*COUNTS_PER_INCH, 0.35, 0.15, 225, DEFAULT_PID_STRAFE, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                drive.stop();

                //Pivot to face perimeter wall
                runtime.reset();
                while(opModeIsActive() && runtime.milliseconds() < 2000){
                    drive.pivot(225, 180, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 500, 5,
                            Direction.FASTEST);
                    telemetry.addData("Status", "Drive Pivoting");
                    telemetry.update();
                }
                drive.stop();

                //Drive towards perimeter wall
                drive.softResetEncoder();
                while(drive.move(drive.getEncoderDistance(), 45*COUNTS_PER_INCH, 22.5*COUNTS_PER_INCH, 0,
                        45*COUNTS_PER_INCH, 0.35, 0.15, -45, DEFAULT_PID_STRAFE, 225, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                drive.stop();

                //Drive to alliance depot to deposit team marker

                break;
        }

        while (opModeIsActive()){
            drive.stop();
            telemetry.addData("Status", "Program Finished");
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
        while(opModeIsActive() && timer.milliseconds() < milliseconds);
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

    private boolean goToPosition(double targetX, double targetY, double targetOrientation, double maxPower, double minPower, double travelDistance){

        double xDistance = targetX - x;
        double yDistance = targetY - y;

        double distance = Math.sqrt((Math.pow(yDistance, 2) + Math.pow(xDistance, 2)));

        double moveAngle = 0;
        moveAngle = Math.toDegrees(Math.atan(xDistance/yDistance));
        if((xDistance < 0 && yDistance < 0) || (xDistance < 0 && yDistance > 0)){
            moveAngle += 180;
        }
        moveAngle = (moveAngle % 360);

        if(drive.move(drive.getEncoderDistance(), travelDistance, distance/0.75, 0, distance, maxPower, minPower,
                moveAngle, DEFAULT_PID, targetOrientation, DEFAULT_ERROR_DISTANCE, 500)
                && !(Math.abs(yDistance) < 0.5 * COUNTS_PER_INCH && Math.abs(xDistance) < 0.5 * COUNTS_PER_INCH)){
            telemetry.addData("Distance", distance/COUNTS_PER_INCH);
            telemetry.addData("X Distance", xDistance/COUNTS_PER_INCH);
            telemetry.addData("Y Distance", yDistance/COUNTS_PER_INCH);
            telemetry.addData("Move Angle", moveAngle);
            return true;
        }else{
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
        double n = horizontalChange;
        x = x + (p*Math.sin(angle) + n*Math.cos(angle));
        y = y + -(p*Math.cos(angle) - n*Math.sin(angle));

        prevLeft = vlPos;
        prevRight = vrPos;
        prevHorizontal = hPos;

        //telemetry.addData("Vertical Right Position", vrPos);
        //telemetry.addData("Vertical Left Position", vlPos);
        //telemetry.addData("Horizontal Position", hPos);
        //telemetry.addData("Angle Radians", angle);
        //telemetry.addData("Angle (Degrees)", Math.toDegrees(angle) % 360);
        telemetry.addData("X Position", x / COUNTS_PER_INCH);
        telemetry.addData("Y Position", y / COUNTS_PER_INCH);
    }

}

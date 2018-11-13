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
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.omnidirectional.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

import java.io.File;
import java.util.ArrayList;

/**
 * Created by Sarthak on 10/26/2018.
 */
@Autonomous(name = "Primary Autonomous Architected", group = "Autonomous")
public class RoverRuckusPrimaryAutonomousArchitected extends LinearOpMode {

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
    double x = 85.5*COUNTS_PER_INCH, y = 85.5*COUNTS_PER_INCH, angle = -(Math.PI/4);

    double prevRight = 0, prevLeft = 0, prevHorizontal = 0;
    double length = 12.75 * COUNTS_PER_INCH;
    final double alpha = 20.63;

    double changeInPosition = 0, changeInAngle = 0;
    double changeInX = 0, changeInY = 0;

    SoundPool sound;
    int beepID;

    double[][] scanPosition;

    double[][] backMineralPosition;

    double[][] middleMineralPosition;

    double[][] frontMineralPosition;

    double[][] depot1;

    double[][] depot2;

    double[][] depot3;

    double[][] crater1;

    final int X_POS_INDEX = 0;
    final int Y_POS_INDEX = 1;
    final int THETA_INDEX = 2;
    final int MAX_POWER_INDEX = 3;
    final int MIN_POWER_INDEX = 4;

    File scanPositionFile = AppUtil.getInstance().getSettingsFile("scanPosition1.txt");
    File backMineralPositionFile = AppUtil.getInstance().getSettingsFile("backMineralPosition.txt");
    File middleMineralPositionFile = AppUtil.getInstance().getSettingsFile("middleMineralPosition.txt");
    File frontMineralPositionFile = AppUtil.getInstance().getSettingsFile("frontMineralPosition.txt");
    File depot1File = AppUtil.getInstance().getSettingsFile("depot1.txt");
    File depot2File = AppUtil.getInstance().getSettingsFile("depot2.txt");
    File depot3File = AppUtil.getInstance().getSettingsFile("depot3.txt");
    File crater1File = AppUtil.getInstance().getSettingsFile("crater1.txt");

    @Override
    public void runOpMode() throws InterruptedException {

        String fileText = "";
        fileText = ReadWriteFile.readFile(scanPositionFile);
        String[] inputs = fileText.split("~");
        scanPosition = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                scanPosition[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Scan Position File");
        telemetry.update();

        fileText = ReadWriteFile.readFile(backMineralPositionFile);
        inputs = fileText.split("~");
        backMineralPosition = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                backMineralPosition[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Back Mineral Position File");
        telemetry.update();

        fileText = ReadWriteFile.readFile(middleMineralPositionFile);
        inputs = fileText.split("~");
        middleMineralPosition = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                middleMineralPosition[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Middle Mineral Position File");
        telemetry.update();

        fileText = ReadWriteFile.readFile(frontMineralPositionFile);
        inputs = fileText.split("~");
        frontMineralPosition = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                frontMineralPosition[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Front Mineral Position File");
        telemetry.update();

        fileText = ReadWriteFile.readFile(depot1File);
        inputs = fileText.split("~");
        depot1 = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                depot1[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Depot1 Position File");
        telemetry.update();

        fileText = ReadWriteFile.readFile(depot2File);
        inputs = fileText.split("~");
        depot2 = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                depot2[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Depot2 Position File");
        telemetry.update();

        fileText = ReadWriteFile.readFile(depot3File);
        inputs = fileText.split("~");
        depot3 = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                depot3[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Depot3 Position File");
        telemetry.update();

        fileText = ReadWriteFile.readFile(crater1File);
        inputs = fileText.split("~");
        crater1 = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                crater1[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Crater1 Position File");
        telemetry.update();

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
        telemetry.addData("Scanning Position Length", scanPosition.length);
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

        //Lower from the lander

        //Move to the scanning position
        for(int i = 0; i < scanPosition.length; i++){
            double x = scanPosition[i][X_POS_INDEX];
            double y = scanPosition[i][Y_POS_INDEX];
            double theta = scanPosition[i][THETA_INDEX];
            double maxPower = scanPosition[i][MAX_POWER_INDEX];
            double minPower = scanPosition[i][MIN_POWER_INDEX];
            while(goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, theta, maxPower, minPower)
                    && opModeIsActive()){
                globalCoordinatePositionUpdate();
                telemetry.addData("Moving to Position", "(" + x +", " + y +")");
                telemetry.addData("Target Angle", theta);
                telemetry.update();
            }
            drive.stop();
            globalCoordinatePositionUpdate();
        }

        //Scan for mineral
        mineralLocation = location.LEFT;

        //Move to the correct mineral position
        switch (mineralLocation){
            case LEFT:
                for(int i = 0; i < backMineralPosition.length; i++){
                    double x = backMineralPosition[i][X_POS_INDEX];
                    double y = backMineralPosition[i][Y_POS_INDEX];
                    double theta = backMineralPosition[i][THETA_INDEX];
                    double maxPower = backMineralPosition[i][MAX_POWER_INDEX];
                    double minPower = backMineralPosition[i][MIN_POWER_INDEX];
                    while(goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, theta, maxPower, minPower)
                            && opModeIsActive()){
                        globalCoordinatePositionUpdate();
                        telemetry.addData("Moving to Position", "(" + x +", " + y +")");
                        telemetry.addData("Target Angle", theta);
                        telemetry.update();
                    }
                    globalCoordinatePositionUpdate();
                }
                break;
            case CENTER:
                for(int i = 0; i < middleMineralPosition.length; i++){
                    double x = middleMineralPosition[i][X_POS_INDEX];
                    double y = middleMineralPosition[i][Y_POS_INDEX];
                    double theta = middleMineralPosition[i][THETA_INDEX];
                    double maxPower = middleMineralPosition[i][MAX_POWER_INDEX];
                    double minPower = middleMineralPosition[i][MIN_POWER_INDEX];
                    while(goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, theta, maxPower, minPower)
                            && opModeIsActive()){
                        telemetry.addData("Moving to Position", "(" + x + ", " + y + ")");
                        telemetry.addData("Target Angle", theta);
                        telemetry.update();
                    }
                    globalCoordinatePositionUpdate();
                }
                break;
            case RIGHT:
                for(int i = 0; i < frontMineralPosition.length; i++){
                    double x = frontMineralPosition[i][X_POS_INDEX];
                    double y = frontMineralPosition[i][Y_POS_INDEX];
                    double theta = frontMineralPosition[i][THETA_INDEX];
                    double maxPower = frontMineralPosition[i][MAX_POWER_INDEX];
                    double minPower = frontMineralPosition[i][MIN_POWER_INDEX];
                    while(goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, theta, maxPower, minPower)
                            && opModeIsActive()){
                        globalCoordinatePositionUpdate();
                        telemetry.addData("Moving to Position", "(" + x +", " + y +")");
                        telemetry.addData("Target Angle", theta);
                        telemetry.update();
                    }
                    globalCoordinatePositionUpdate();
                }
                break;
        }
        drive.stop();
        globalCoordinatePositionUpdate();

        //Move to the depot
        switch (mineralLocation){
            case LEFT:
                for(int i = 0; i < depot1.length; i++){
                    double x = depot1[i][X_POS_INDEX];
                    double y = depot1[i][Y_POS_INDEX];
                    double theta = depot1[i][THETA_INDEX];
                    double maxPower = depot1[i][MAX_POWER_INDEX];
                    double minPower = depot1[i][MIN_POWER_INDEX];
                    while(goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, theta, maxPower, minPower)
                            && opModeIsActive()){
                        globalCoordinatePositionUpdate();
                        telemetry.addData("Moving to Position", "(" + x +", " + y +")");
                        telemetry.addData("Target Angle", theta);
                        telemetry.update();
                    }
                    globalCoordinatePositionUpdate();
                }
                break;
            case CENTER:
                for(int i = 0; i < depot2.length; i++){
                    double x = depot2[i][X_POS_INDEX];
                    double y = depot2[i][Y_POS_INDEX];
                    double theta = depot2[i][THETA_INDEX];
                    double maxPower = depot2[i][MAX_POWER_INDEX];
                    double minPower = depot2[i][MIN_POWER_INDEX];
                    while(goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, theta, maxPower, minPower)
                            && opModeIsActive()){
                        telemetry.addData("Moving to Position", "(" + x + ", " + y + ")");
                        telemetry.addData("Target Angle", theta);
                        telemetry.update();
                    }
                    globalCoordinatePositionUpdate();
                }
                break;
            case RIGHT:
                for(int i = 0; i < depot3.length; i++){
                    double x = depot3[i][X_POS_INDEX];
                    double y = depot3[i][Y_POS_INDEX];
                    double theta = depot3[i][THETA_INDEX];
                    double maxPower = depot3[i][MAX_POWER_INDEX];
                    double minPower = depot3[i][MIN_POWER_INDEX];
                    while(goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, theta, maxPower, minPower)
                            && opModeIsActive()){
                        globalCoordinatePositionUpdate();
                        telemetry.addData("Moving to Position", "(" + x +", " + y +")");
                        telemetry.addData("Target Angle", theta);
                        telemetry.update();
                    }
                    globalCoordinatePositionUpdate();
                }
                break;
        }
        drive.stop();
        globalCoordinatePositionUpdate();

        //Deposit team marker
        waitMilliseconds(1000, runtime);

        //Go to the crater
        for(int i = 0; i < crater1.length; i++){
            double x = crater1[i][X_POS_INDEX];
            double y = crater1[i][Y_POS_INDEX];
            double theta = crater1[i][THETA_INDEX];
            double maxPower = crater1[i][MAX_POWER_INDEX];
            double minPower = crater1[i][MIN_POWER_INDEX];
            while(goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, theta, maxPower, minPower)
                    && opModeIsActive()){
                globalCoordinatePositionUpdate();
                telemetry.addData("Moving to Position", "(" + x +", " + y +")");
                telemetry.addData("Target Angle", theta);
                telemetry.update();
            }
            drive.stop();
            globalCoordinatePositionUpdate();
        }

        //sound.play(beepID, 1, 1, 1, 0, 1);
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
        right_front = hardwareMap.dcMotor.get("rf");
        right_back = hardwareMap.dcMotor.get("rb");
        left_front = hardwareMap.dcMotor.get("lf");
        left_back = hardwareMap.dcMotor.get("lb");

        verticalLeft = hardwareMap.dcMotor.get("rf");
        verticalRight = hardwareMap.dcMotor.get("rb");
        horizontal = hardwareMap.dcMotor.get("lf");
        horizontal2 = hardwareMap.dcMotor.get("lb");

        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
        moveAngle = -((moveAngle % 360) - Math.toDegrees(angle));

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

        //Change in angle
        changeInAngle = (leftChange - rightChange) / (length);
        angle = ((angle + changeInAngle));

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange + (((leftChange-rightChange)/2) * Math.sin(alpha));
        x = x + (p*Math.cos(angle) - n*Math.sin(angle));
        y = y + (p*Math.sin(angle) + n*Math.cos(angle));

        prevLeft = vlPos;
        prevRight = vrPos;
        prevHorizontal = hPos;

        //telemetry.addData("Vertical Right Position", vrPos);
        //telemetry.addData("Vertical Left Position", vlPos);
        //telemetry.addData("Horizontal Position", hPos);
        //telemetry.addData("Angle Radians", angle);
        //telemetry.addData("Algorithm 2 Angle (Degrees)", Math.toDegrees(angle) % 360);
        telemetry.addData("X Position", x / COUNTS_PER_INCH);
        telemetry.addData("Y Position", y / COUNTS_PER_INCH);
    }

    public double distanceFormula(double x, double y){
        double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
        return distance;
    }

}

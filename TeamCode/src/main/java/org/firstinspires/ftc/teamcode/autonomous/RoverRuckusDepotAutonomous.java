package org.firstinspires.ftc.teamcode.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.omnidirectional.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.ITeamMarker;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.ServoArmDrop;

import java.io.File;
import java.util.ArrayList;

/**
 * Created by Sarthak on 10/26/2018.
 */
@Autonomous(name = "\uD83E\uDD16 Depot Autonomous", group = "Autonomous")
public class RoverRuckusDepotAutonomous extends LinearOpMode {

    IDrivetrain drive;
    DcMotor right_front, right_back, left_front, left_back;
    DcMotor verticalLeft, verticalRight, horizontal, horizontal2;
    DcMotor mineral_rotation;
    ArrayList motors, encoders;

    ITeamMarker teamMarker;
    Servo team_marker;

    Servo scanner;
    Servo hang_latch;

    DigitalChannel rotation_limit;

    DcMotor hang;

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
    location detectedLocation;

    //Create detector to be used for the gold mineral
    private GoldDetector genericDetector = null;

    //define constants for drive movement parameters
    final double DEFAULT_MAX_POWER = .75;
    final double DEFAULT_MIN_POWER = .35;
    final double DEFAULT_MIN_POWER_PIVOT = .15;

    final double DEFAULT_ERROR_DISTANCE = 10;

    final double[] DEFAULT_PID = {.02};
    final double COUNTS_PER_INCH = 307.699557;

    //Position variables
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0;
    double robotGlobalXPosition = 0, robotGlobalYPosition = 0, robotOrientationRadians = 0;

    double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;
    double robotEncoderWheelDistance = 12.75 * COUNTS_PER_INCH;
    final double normalEncoderWheelPositionAngleFromRotationAxis = 20.63;

    double changeInRobotOrientation = 0;

    ModernRoboticsI2cRangeSensor rightWallPing;

    final int X_POS_INDEX = 0;
    final int Y_POS_INDEX = 1;
    final int THETA_INDEX = 2;
    final int MAX_POWER_INDEX = 3;
    final int MIN_POWER_INDEX = 4;

    double[][] leftMineral;
    double[][] centerMineral;
    double[][] rightMineral;
    double[][] depot;

    File leftMineralPositionFile = AppUtil.getInstance().getSettingsFile("leftMineral.txt");
    File middleMineralPositionFile = AppUtil.getInstance().getSettingsFile("centerMineral.txt");
    File rightMineralPositionFile = AppUtil.getInstance().getSettingsFile("rightMineral.txt");
    File depotFile = AppUtil.getInstance().getSettingsFile("depot.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        //Init motor hardware map and behaviors
        setMotorBehaviors();

        telemetry.addData("Status", "Read Scan Position File");
        telemetry.update();

        String fileText = ReadWriteFile.readFile(leftMineralPositionFile);
        String[] inputs = fileText.split("~");
        leftMineral = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                leftMineral[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Left Mineral Position File");
        telemetry.update();

        fileText = ReadWriteFile.readFile(middleMineralPositionFile);
        inputs = fileText.split("~");
        centerMineral = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                centerMineral[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Center Mineral Position File");
        telemetry.update();

        fileText = ReadWriteFile.readFile(rightMineralPositionFile);
        inputs = fileText.split("~");
        rightMineral = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                rightMineral[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Right Mineral Position File");
        telemetry.update();

        fileText = ReadWriteFile.readFile(depotFile);
        inputs = fileText.split("~");
        depot = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                depot[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Depot Position File");
        telemetry.update();

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

        teamMarker.hold();

        waitForStart();
        runtime.reset();

        /**
         * *****************************************************************************************
         * *****************************************************************************************
         * *******************************OPMODE RUNS HERE******************************************
         * *****************************************************************************************
         * *****************************************************************************************
         */
        hang_latch.setPosition(1);

        waitMilliseconds(250, runtime);

        hang.setTargetPosition(9600);
        hang.setPower(1);
        runtime.reset();
        while(hang.isBusy() && opModeIsActive() && runtime.milliseconds() < 6000){
            telemetry.addData("hang pos", hang.getCurrentPosition());
            telemetry.update();
        }

        hang.setPower(0);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        genericDetector.enable();
        runtime.reset();

        mineral_rotation.setTargetPosition(200);
        mineral_rotation.setPower(1);

        waitMilliseconds(1000, runtime);

        mineral_rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(opModeIsActive() && rotation_limit.getState()){
            mineral_rotation.setPower(-0.15);
        }
        mineral_rotation.setPower(0);

        while(runtime.milliseconds() < 1250 && opModeIsActive());

        //Scan for mineral
        globalCoordinatePositionUpdate();
        boolean found = genericDetector.isFound();
        if(found && (genericDetector.getScreenPosition().x < 375)){
            mineralLocation = location.CENTER;
        }else if (found && genericDetector.getScreenPosition().x > 450) {
            mineralLocation = location.RIGHT;
        }
        else{
            while(opModeIsActive() && scanner.getPosition() < 0.6){
                scanner.setPosition(scanner.getPosition() + 0.001);
                telemetry.addData("Y Position", genericDetector.getScreenPosition().y);
                telemetry.update();
            }
            runtime.reset();
            while(runtime.milliseconds() < 1000 && opModeIsActive()){
                telemetry.addData("Y Position", genericDetector.getScreenPosition().y);
                telemetry.update();
            }

            found = genericDetector.isFound();
            if(found && genericDetector.getScreenPosition().x > 100 && genericDetector.getScreenPosition().y > 40) {
                mineralLocation = location.RIGHT;
            }else {
                mineralLocation = location.LEFT;
            }
        }
        globalCoordinatePositionUpdate();
        genericDetector.disable();
        scanner.setPosition(0.5);
        team_marker.setPosition(0.5);

        switch (mineralLocation){
            case LEFT:
                for(int i = 0; i < leftMineral.length; i++){
                    double x = leftMineral[i][X_POS_INDEX];
                    double y = leftMineral[i][Y_POS_INDEX];
                    double theta = leftMineral[i][THETA_INDEX];
                    double maxPower = leftMineral[i][MAX_POWER_INDEX];
                    double minPower = leftMineral[i][MIN_POWER_INDEX];
                    while(goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, theta, maxPower, minPower)
                            && opModeIsActive()){
                        globalCoordinatePositionUpdate();
                        telemetry.addData("Moving to Position", "(" + x +", " + y +")");
                        telemetry.addData("Target Angle", theta);
                        telemetry.update();
                    }
                    drive.stop();
                    globalCoordinatePositionUpdate();
                    if(i == 0){
                        waitMilliseconds(1000, runtime);
                    }
                }
                break;
            case CENTER:
                for(int i = 0; i < centerMineral.length; i++){
                    double x = centerMineral[i][X_POS_INDEX];
                    double y = centerMineral[i][Y_POS_INDEX];
                    double theta = centerMineral[i][THETA_INDEX];
                    double maxPower = centerMineral[i][MAX_POWER_INDEX];
                    double minPower = centerMineral[i][MIN_POWER_INDEX];
                    while(goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, theta, maxPower, minPower)
                            && opModeIsActive()){
                        globalCoordinatePositionUpdate();
                        telemetry.addData("Moving to Position", "(" + x + ", " + y + ")");
                        telemetry.addData("Target Angle", theta);
                        telemetry.update();
                    }
                    drive.stop();
                    globalCoordinatePositionUpdate();
                    if(i == 0){
                        waitMilliseconds(1000, runtime);
                    }
                }
                break;
            case RIGHT:
                for(int i = 0; i < rightMineral.length; i++){
                    double x = rightMineral[i][X_POS_INDEX];
                    double y = rightMineral[i][Y_POS_INDEX];
                    double theta = rightMineral[i][THETA_INDEX];
                    double maxPower = rightMineral[i][MAX_POWER_INDEX];
                    double minPower = rightMineral[i][MIN_POWER_INDEX];
                    while(goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, theta, maxPower, minPower)
                            && opModeIsActive()){
                        globalCoordinatePositionUpdate();
                        telemetry.addData("Moving to Position", "(" + x +", " + y +")");
                        telemetry.addData("Target Angle", theta);
                        telemetry.update();
                    }
                    drive.stop();
                    globalCoordinatePositionUpdate();
                    if(i == 0){
                        waitMilliseconds(1000, runtime);
                    }
                }
                break;
        }
        drive.stop();
        globalCoordinatePositionUpdate();

        //hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //hang.setTargetPosition(0);
        //hang.setPower(1);

        for(int i = 0; i < depot.length; i++){
            double x = depot[i][X_POS_INDEX];
            double y = depot[i][Y_POS_INDEX];
            double theta = depot[i][THETA_INDEX];
            double maxPower = depot[i][MAX_POWER_INDEX];
            double minPower = depot[i][MIN_POWER_INDEX];
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

        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 1000){
            drive.pivot(135, 120, 1, 0.35, 50, 2, Direction.FASTEST);
            globalCoordinatePositionUpdate();
            telemetry.update();
        }
        drive.stop();
        globalCoordinatePositionUpdate();

        double wallReading = rightWallPing.cmUltrasonic();
        while (wallReading == 255){
            wallReading = rightWallPing.cmUltrasonic();
        }

        double wallCorrection = (10 - wallReading) / 2.54;
        if(wallCorrection > 0.75){
            drive.softResetEncoder();
            while(opModeIsActive() && drive.move(drive.getEncoderDistance(), wallCorrection*COUNTS_PER_INCH, wallCorrection*COUNTS_PER_INCH,
                    0, wallCorrection*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 45 , DEFAULT_PID, 135
                    ,0.5*COUNTS_PER_INCH, 0));
            drive.stop();
        }else if (wallCorrection < -0.75){
            drive.softResetEncoder();
            while(opModeIsActive() && drive.move(drive.getEncoderDistance(), Math.abs(wallCorrection)*COUNTS_PER_INCH, wallCorrection*COUNTS_PER_INCH,
                    0, wallCorrection*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, -135 , DEFAULT_PID, 135
                    ,0.5*COUNTS_PER_INCH, 0));
            drive.stop();
        }

        //Drive to alliance depot
        drive.softResetEncoder();
        while(opModeIsActive() && drive.move(drive.getEncoderDistance(), 44*COUNTS_PER_INCH, 10*COUNTS_PER_INCH,
                0, 30*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, -50 , DEFAULT_PID, 135
                ,0.5*COUNTS_PER_INCH, 0));
        drive.stop();

        //Drop team marker
        teamMarker.drop();
        waitMilliseconds(500, runtime);

        //Drive to crater to park
        drive.softResetEncoder();
        while(opModeIsActive() && drive.move(drive.getEncoderDistance(), 25*COUNTS_PER_INCH, 25*COUNTS_PER_INCH,
                0, 60*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 138 , DEFAULT_PID, 135
                ,0.5*COUNTS_PER_INCH, 0));
        teamMarker.hold();
        while(opModeIsActive() && drive.move(drive.getEncoderDistance(), 62*COUNTS_PER_INCH, 25*COUNTS_PER_INCH,
                0, 60*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 135 , DEFAULT_PID, 135
                ,0.5*COUNTS_PER_INCH, 0));
        drive.stop();

        while (opModeIsActive()){
            drive.stop();
            globalCoordinatePositionUpdate();
            telemetry.addData("Status", "Program Finished");
            telemetry.addData("X Position", robotGlobalXPosition /COUNTS_PER_INCH);
            telemetry.addData("Y Position", robotGlobalYPosition /COUNTS_PER_INCH);
            telemetry.update();
        }

        /*//Begin Sampling
        switch (mineralLocation){
            case CENTER:
                globalCoordinatePositionUpdate();
                while(goToPosition(-16.5*COUNTS_PER_INCH, -2.25*COUNTS_PER_INCH, 0, 1.0, 0.6)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-20, 17)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                runtime.reset();
                globalCoordinatePositionUpdate();
                while(goToPosition(-23.5*COUNTS_PER_INCH, -2.25*COUNTS_PER_INCH, 0, 1.0, 0.4)
                        && opModeIsActive() && runtime.milliseconds() < 1500){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-20, 17)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                while(goToPosition(-16.5*COUNTS_PER_INCH, -2.25*COUNTS_PER_INCH, 0, 1.0, 0.4)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-20, 17)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                break;
            case LEFT:
                globalCoordinatePositionUpdate();
                while(goToPosition(-3*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0, 0.5, 0.35)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-20, 17)");
                    telemetry.update();
                }
                while(goToPosition(-16.5*COUNTS_PER_INCH, -17*COUNTS_PER_INCH, 0, 0.6, 0.4)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-20, 17)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                runtime.reset();
                globalCoordinatePositionUpdate();
                while(goToPosition(-23.5*COUNTS_PER_INCH, -17*COUNTS_PER_INCH, 0, 1.0, 0.4)
                        && opModeIsActive() && runtime.milliseconds() < 1500){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-20, 17)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();


                break;
            case RIGHT:
                while(goToPosition(-3*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0, 0.5, 0.35)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-20, 17)");
                    telemetry.update();
                }
                while(goToPosition(-16.5*COUNTS_PER_INCH, 13.5*COUNTS_PER_INCH, 0, 1.0, 0.6)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-20, 17)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                runtime.reset();
                globalCoordinatePositionUpdate();
                while(goToPosition(-23.5*COUNTS_PER_INCH, 13.5*COUNTS_PER_INCH, 0, 1.0, 0.4)
                        && opModeIsActive() && runtime.milliseconds() < 1500){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-20, 17)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                while(goToPosition(-16.5*COUNTS_PER_INCH, 13.5*COUNTS_PER_INCH, 0, 1.0, 0.4)
                        && opModeIsActive()){
                    globalCoordinatePositionUpdate();
                    telemetry.addData("Moving to Position", "(-20, 17)");
                    telemetry.update();
                }
                drive.stop();
                globalCoordinatePositionUpdate();

                break;
        }

        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setTargetPosition(0);
        hang.setPower(1);

        globalCoordinatePositionUpdate();
        while(goToPosition(-15*COUNTS_PER_INCH, -44*COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, 0.4)
                && opModeIsActive()){
            globalCoordinatePositionUpdate();
            telemetry.addData("Moving to Position", "(-20, 17)");
            telemetry.update();
        }
        drive.stop();
        globalCoordinatePositionUpdate();

        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 1000){
            drive.pivot(135, 120, 1, 0.35, 50, 2, Direction.FASTEST);
            globalCoordinatePositionUpdate();
            telemetry.update();
        }
        drive.stop();
        globalCoordinatePositionUpdate();

        /*double wallDistance = rightWallPing.cmUltrasonic()
        wallDistance = 12-wallDistance;
        double angleCorrection = Math.toDegrees(Math.asin(wallDistance/44));

        //Drive to alliance depot
        drive.softResetEncoder();
        while(opModeIsActive() && drive.move(drive.getEncoderDistance(), 44*COUNTS_PER_INCH, 10*COUNTS_PER_INCH,
                0, 30*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, -50 , DEFAULT_PID, 135
                ,0.5*COUNTS_PER_INCH, 0));
        drive.stop();

        //Drop team marker
        teamMarker.drop();
        waitMilliseconds(2500, runtime);

        hang.setPower(0);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Drive to crater to park
        drive.softResetEncoder();
        while(opModeIsActive() && drive.move(drive.getEncoderDistance(), 25*COUNTS_PER_INCH, 25*COUNTS_PER_INCH,
                0, 60*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 138 , DEFAULT_PID, 135
                ,0.5*COUNTS_PER_INCH, 0));
        teamMarker.hold();
        while(opModeIsActive() && drive.move(drive.getEncoderDistance(), 62*COUNTS_PER_INCH, 25*COUNTS_PER_INCH,
                0, 60*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 135 , DEFAULT_PID, 135
                ,0.5*COUNTS_PER_INCH, 0));
        drive.stop();

        while (opModeIsActive()){
            drive.stop();
            globalCoordinatePositionUpdate();
            telemetry.addData("Status", "Program Finished");
            telemetry.addData("X Position", robotGlobalXPosition/COUNTS_PER_INCH);
            telemetry.addData("Y Position", robotGlobalYPosition/COUNTS_PER_INCH);
            telemetry.update();
        }*/

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

        mineral_rotation = hardwareMap.dcMotor.get("mineral_rotation");
        mineral_rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineral_rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineral_rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        team_marker = hardwareMap.servo.get("marker_servo");
        teamMarker = new ServoArmDrop(team_marker);

        scanner = hardwareMap.servo.get("scanner");
        scanner.setPosition(0.5);

        rightWallPing = (ModernRoboticsI2cRangeSensor) hardwareMap.get("right_us");

        hang_latch = hardwareMap.servo.get("hang_stopper");
        hang_latch.setPosition(0);

        rotation_limit = hardwareMap.digitalChannel.get("rotation_limit");

        hang = hardwareMap.dcMotor.get("hang");
        hang.setDirection(DcMotorSimple.Direction.REVERSE);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        double xDistance = targetX - robotGlobalXPosition;
        double yDistance = targetY - robotGlobalYPosition;

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
        moveAngle = (moveAngle % 360);

//        data.addField((float) robotGlobalXPosition);
//        data.addField((float) robotGlobalYPosition);
//        data.addField((float) orientationDifference);
//        data.addField((float) xDistance);
//        data.addField((float) yDistance);
//        data.addField((float) moveAngle);
//        data.addField((float) power);
//        data.addField((float) distance);
//        data.newLine();

        if(!(Math.abs(yDistance) < 0.75 * COUNTS_PER_INCH && Math.abs(xDistance) < 0.75 * COUNTS_PER_INCH
                && Math.abs(orientationDifference) < 5)){
            drive.move(0, distance, distance, 0, distance, power, power,
                    moveAngle, DEFAULT_PID, targetOrientation, DEFAULT_ERROR_DISTANCE, 500);
            telemetry.addData("Distance", distance/COUNTS_PER_INCH);
            telemetry.addData("X Distance", xDistance/COUNTS_PER_INCH);
            telemetry.addData("Y Distance", yDistance/COUNTS_PER_INCH);
            telemetry.addData("Move Angle", moveAngle);

            return true;
        }else{
            notReset = false;
            return false;
        }

    }

    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        verticalLeftEncoderWheelPosition = verticalLeft.getCurrentPosition();
        verticalRightEncoderWheelPosition = -verticalRight.getCurrentPosition();
        normalEncoderWheelPosition = horizontal.getCurrentPosition();

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;
        double horizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;

        //Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = (robotOrientationRadians + changeInRobotOrientation);

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange + (((leftChange-rightChange)/2) * Math.sin(normalEncoderWheelPositionAngleFromRotationAxis));
        robotGlobalXPosition = robotGlobalXPosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYPosition = robotGlobalYPosition + -(p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;

        telemetry.addData("X Position", robotGlobalXPosition / COUNTS_PER_INCH);
        telemetry.addData("Y Position", robotGlobalYPosition / COUNTS_PER_INCH);
    }

    public double distanceFormula(double x, double y){
        double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
        return distance;
    }

}
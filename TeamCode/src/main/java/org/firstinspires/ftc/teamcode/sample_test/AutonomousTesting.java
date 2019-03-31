package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.omnidirectional.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

import java.io.File;
import java.util.ArrayList;

/**
 * Created by Sarthak on 10/29/2018.
 */
@Autonomous(name = "Autonomous Testing", group = "Autonomous")
public class AutonomousTesting extends LinearOpMode {
    IDrivetrain drive;
    DcMotor right_front, right_back, left_front, left_back;
    DcMotor mineral_rotation, mineralExtension;
    DcMotor verticalLeft, verticalRight, horizontal, horizontal2;
    ArrayList motors, encoders;

    DcMotor intakeRotation;

    CRServo intake;

    DcMotor hang;

    ModernRoboticsI2cRangeSensor leftWallPing, rightWallPing;

    DigitalChannel rotation_limit;

    IIMU imu;
    BNO055IMU boschIMU;

    Servo hang_latch;

    Servo scanner;

    final int ANGLE_INDEX = 0;
    final int RF_LB_INDEX = 2;
    final int LF_RB_INDEX = 1;

    //Define possible mineral locations in enum
    enum location {
        LEFT, CENTER, RIGHT, UNKNOWN
    };

    final double[] DEFAULT_PID = {.035};
    final double COUNTS_PER_INCH = 307.699557;

    //Position variables
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0;
    double robotGlobalXPosition = 0, robotGlobalYPosition = 0, robotOrientationRadians = 0;

    double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;
    double robotEncoderWheelDistance = 12.75 * COUNTS_PER_INCH;
    final double normalEncoderWheelPositionAngleFromRotationAxis = -20.63;

    double changeInRobotOrientation = 0;

    final int X_POS_INDEX = 0;
    final int Y_POS_INDEX = 1;
    final int THETA_INDEX = 2;
    final int MAX_POWER_INDEX = 3;
    final int MIN_POWER_INDEX = 4;

    double[][] testCoordinates;
    double[][] motorPowerLookup;
    double[][] lowMotorPowerLookup;

    File testCoordinatesFile = AppUtil.getInstance().getSettingsFile("testCoordinates.txt");
    File motorPowersFile = AppUtil.getInstance().getSettingsFile("Drivetrain Motor Powers.txt");
    File lowMotorPowerFile = AppUtil.getInstance().getSettingsFile("Drivetrain Motor Power Medium.txt");

    @Override
    public void runOpMode() throws InterruptedException {

        String fileText = ReadWriteFile.readFile(testCoordinatesFile);
        String[] inputs = fileText.split("~");
        testCoordinates = new double[inputs.length][5];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                testCoordinates[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Test Position File");
        telemetry.update();

        fileText = ReadWriteFile.readFile(motorPowersFile);
        inputs = fileText.split("~");
        motorPowerLookup = new double[inputs.length][3];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                motorPowerLookup[i][j] = Double.parseDouble(params[j]);
            }
        }

        fileText = ReadWriteFile.readFile(lowMotorPowerFile);
        inputs = fileText.split("~");
        lowMotorPowerLookup = new double[inputs.length][3];
        for(int i = 0; i < inputs.length; i++){
            String[] params = inputs[i].split(",");
            for(int j = 0; j < params.length; j++){
                lowMotorPowerLookup[i][j] = Double.parseDouble(params[j]);
            }
        }

        telemetry.addData("Status", "Read Motor Power File");
        telemetry.update();

        //Init motor hardware map and behaviors
        setMotorBehaviors();

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
        telemetry.addData("Move Angle", motorPowerLookup[0][0]);
        telemetry.addData("LF_RB Power", motorPowerLookup[0][1]);
        telemetry.addData("RF_LB Power", motorPowerLookup[0][2]);
        telemetry.update();

        waitForStart();

        /**
         * *****************************************************************************************
         * *****************************************************************************************
         * *******************************OPMODE RUNS HERE******************************************
         * *****************************************************************************************
         * *****************************************************************************************
         */

        for(int i = 0; i < testCoordinates.length; i++){
            double x = testCoordinates[i][X_POS_INDEX];
            double y = testCoordinates[i][Y_POS_INDEX];
            double theta = testCoordinates[i][THETA_INDEX];
            double maxPower = testCoordinates[i][MAX_POWER_INDEX];
            double minPower = testCoordinates[i][MIN_POWER_INDEX];
            while(goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, theta, maxPower, minPower)
                    && opModeIsActive()){
                globalCoordinatePositionUpdate();
                telemetry.addData("Moving to Position", "(" + x +", " + y +")");
                telemetry.addData("Target Angle", theta);
                telemetry.update();
            }
            drive.stop();
            globalCoordinatePositionUpdate();
            while(gamepad1.atRest() && gamepad2.atRest() && opModeIsActive()){
                telemetry.addData("Status", "Move Joysticks to Move to Next Position");
                telemetry.update();
            }
        }

        drive.stop();

        while (opModeIsActive()){
            drive.stop();
            globalCoordinatePositionUpdate();
            telemetry.update();
        }

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
        right_front = hardwareMap.dcMotor.get("rf");
        right_back = hardwareMap.dcMotor.get("rb");
        left_front = hardwareMap.dcMotor.get("lf");
        left_back = hardwareMap.dcMotor.get("lb");

        verticalLeft = hardwareMap.dcMotor.get("rf");
        verticalRight = hardwareMap.dcMotor.get("rb");
        horizontal = hardwareMap.dcMotor.get("lf");
        horizontal2 = hardwareMap.dcMotor.get("lb");

        intakeRotation = hardwareMap.dcMotor.get("intake_rotation");
        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mineral_rotation = hardwareMap.dcMotor.get("mineral_rotation");
        mineral_rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineral_rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineral_rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mineralExtension = hardwareMap.dcMotor.get("mineral_extension");
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        scanner = hardwareMap.servo.get("scanner");
        scanner.setPosition(0.5);

        rotation_limit = hardwareMap.digitalChannel.get("rotation_limit");

        hang_latch = hardwareMap.servo.get("hang_stopper");
        hang_latch.setPosition(0);

        leftWallPing = (ModernRoboticsI2cRangeSensor) hardwareMap.get("left_us");
        rightWallPing = (ModernRoboticsI2cRangeSensor) hardwareMap.get("right_us");

        intake = hardwareMap.crservo.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        hang = hardwareMap.dcMotor.get("hang");
        //hang.setDirection(DcMotorSimple.Direction.REVERSE);
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

        double distance = distanceFormula(xDistance, yDistance);

        double robotOrientationDifference = targetOrientation - imu.getZAngle();

        double robotMoveAngle;
        robotMoveAngle = Math.toDegrees(Math.atan(xDistance/yDistance));
        if((xDistance < 0 && yDistance < 0) || (xDistance > 0 && yDistance < 0)){
            robotMoveAngle += 180;
        }
        robotMoveAngle = (robotMoveAngle % 360);

        if(!(Math.abs(yDistance) < 0.75 * COUNTS_PER_INCH && Math.abs(xDistance) < 0.75 * COUNTS_PER_INCH
                && Math.abs(robotOrientationDifference) < 5)){
            double currentAngle = imu.getZAngle(targetOrientation);

            double[] currentMotorPowers = null;
            double lfrbPower = 0, rflbPower = 0;
            if(distance > 3*COUNTS_PER_INCH){
                currentMotorPowers = getMotorPowers(robotMoveAngle);
                double pivotCorrection = ((currentAngle - targetOrientation) * DEFAULT_PID[0]);
                lfrbPower = (currentMotorPowers[0] - pivotCorrection);
                rflbPower = (currentMotorPowers[1] + pivotCorrection);
            }else{
                currentMotorPowers = getMotorPowers(robotMoveAngle);
                double pivotCorrection = ((currentAngle - targetOrientation) * DEFAULT_PID[0]) * 0.5;
                lfrbPower = (currentMotorPowers[0] - pivotCorrection) * 0.5;
                rflbPower = (currentMotorPowers[1] + pivotCorrection) * 0.5;
            }


            right_front.setPower(rflbPower);
            left_back.setPower(rflbPower);
            left_front.setPower(lfrbPower);
            right_back.setPower(lfrbPower);

            telemetry.addData("Encoder Distance", distance/COUNTS_PER_INCH);
            telemetry.addData("X Distance", xDistance/COUNTS_PER_INCH);
            telemetry.addData("Y Distance", yDistance/COUNTS_PER_INCH);
            telemetry.addData("Move Angle", robotMoveAngle);

            return true;
        }else{
            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

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

    public double[] getMotorPowers(double moveAngle){
        double[] motorPowers = new double[2];

        double rflbPower = 0;
        double lfrbPower = 0;

        int lowerIndex = 0;
        int higherIndex = 0;

        boolean exactAngle = false;
        int index = 0;

        for(int i = 0; i < motorPowerLookup.length; i++) {
            double currentMoveAngle = motorPowerLookup[i][ANGLE_INDEX];
            if(moveAngle == currentMoveAngle) {
                exactAngle = true;
                rflbPower = motorPowerLookup[i][RF_LB_INDEX];
                lfrbPower = motorPowerLookup[i][LF_RB_INDEX];
                break;
            }else if(moveAngle > currentMoveAngle) {
                lowerIndex = i;
            }else {
                higherIndex = i;
                break;
            }
        }

        if(!exactAngle){
            double lowRFLBPower = motorPowerLookup[lowerIndex][RF_LB_INDEX];
            double highRFLBPower = motorPowerLookup[higherIndex][RF_LB_INDEX];

            double lowLFRBPower = motorPowerLookup[lowerIndex][LF_RB_INDEX];
            double highLFRBPower = motorPowerLookup[higherIndex][LF_RB_INDEX];

            double lowAngle = motorPowerLookup[lowerIndex][ANGLE_INDEX];
            double highAngle = motorPowerLookup[higherIndex][ANGLE_INDEX];

            rflbPower = ((moveAngle - lowAngle)*(highRFLBPower - lowRFLBPower)/(highAngle - lowAngle)) + lowRFLBPower;
            lfrbPower = ((moveAngle - lowAngle)*(highLFRBPower - lowLFRBPower)/(highAngle - lowAngle)) + lowLFRBPower;
        }

        motorPowers[0] = rflbPower;
        motorPowers[1] = lfrbPower;

        return motorPowers;
    }

    public double[] getLowMotorPowers(double moveAngle){
        double[] motorPowers = new double[2];

        double rflbPower = 0;
        double lfrbPower = 0;

        int lowerIndex = 0;
        int higherIndex = 0;

        boolean exactAngle = false;
        int index = 0;

        for(int i = 0; i < lowMotorPowerLookup.length; i++) {
            double currentMoveAngle = lowMotorPowerLookup[i][ANGLE_INDEX];
            if(moveAngle == currentMoveAngle) {
                exactAngle = true;
                rflbPower = lowMotorPowerLookup[i][RF_LB_INDEX];
                lfrbPower = lowMotorPowerLookup[i][LF_RB_INDEX];
                break;
            }else if(moveAngle > currentMoveAngle) {
                lowerIndex = i;
            }else {
                higherIndex = i;
                break;
            }
        }

        if(!exactAngle){
            double lowRFLBPower = lowMotorPowerLookup[lowerIndex][RF_LB_INDEX];
            double highRFLBPower = lowMotorPowerLookup[higherIndex][RF_LB_INDEX];

            double lowLFRBPower = lowMotorPowerLookup[lowerIndex][LF_RB_INDEX];
            double highLFRBPower = lowMotorPowerLookup[higherIndex][LF_RB_INDEX];

            double lowAngle = lowMotorPowerLookup[lowerIndex][ANGLE_INDEX];
            double highAngle = lowMotorPowerLookup[higherIndex][ANGLE_INDEX];

            rflbPower = ((moveAngle - lowAngle)*(highRFLBPower - lowRFLBPower)/(highAngle - lowAngle)) + lowRFLBPower;
            lfrbPower = ((moveAngle - lowAngle)*(highLFRBPower - lowLFRBPower)/(highAngle - lowAngle)) + lowLFRBPower;
        }

        motorPowers[0] = rflbPower;
        motorPowers[1] = lfrbPower;

        return motorPowers;
    }

    public double distanceFormula(double x, double y){
        double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
        return distance;
    }

}

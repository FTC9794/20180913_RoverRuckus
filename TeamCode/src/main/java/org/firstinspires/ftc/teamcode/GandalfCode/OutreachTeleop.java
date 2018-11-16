package org.firstinspires.ftc.teamcode.GandalfCode;

//import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputController;
import com.qualcomm.robotcore.hardware.PWMOutputControllerEx;
import com.qualcomm.robotcore.hardware.PWMOutputImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;

/**
 * Created by Raashi Dewan on 1/7/2018.
 */

@TeleOp(group = "Outreach", name = "Outreach Gandalf Teleop")
@Disabled
public class OutreachTeleop extends LinearOpMode{

    //DC motors
    DcMotor rf;
    DcMotor rb;
    DcMotor lf;
    DcMotor lb;
    DcMotor shooter;
    DcMotor sweeper;
    DcMotor led;
    //Sensors
    ModernRoboticsAnalogOpticalDistanceSensor light;
    ModernRoboticsI2cRangeSensor ultrasonic;
    ModernRoboticsI2cGyro gyro;
    ColorSensor colorSensor;
    Orientation q;
    //AHRS navx_device;

    //Servos
    Servo shooterGate;
    Servo flag;
    boolean harvesterLetGo = true;
    DigitalChannel cap_top_limit;

    //Device interface modules
    DeviceInterfaceModule deviceModule1;
    DeviceInterfaceModule deviceModule2;


    //servo controller
    ServoController servoController;

    //shooting speed variables
    double shooterLowRange = 1900;
    double shooterHighRange = 2100;
    double shooterSpeed;
    int shooterMaxSpeed = 4600;
    double shootingSpeed = 0.775;
    double previousEncoder;
    double currentEncoder;
    ElapsedTime shooterSpeedTimer;

    //drivetrain
    MecanumDrive drive;

    final double gearsDesiredX = -130;
    final double gearsDesiredZ = -520;

    //toggle variables
    ElapsedTime shooterToggleTimer;
    ElapsedTime harvestToggleTimer;
    boolean shooterToggle;
    boolean harvestToggle;
    boolean pressed = true;
    boolean reachedPivot = false;
    //Movement variables
    ElapsedTime timer;

    //game timers
    ElapsedTime shooterTime;
    ElapsedTime gameTime;

    //motion variables with controller
    double speed;
    double angle;
    double IMUAngle;
    double pivotSpeed;
    double offset;

    //motion parameters
    double proportionalPower;
    double distanceError;
    double countsPerInch = 119.4;

    //cap ball positions and speeds
    double highPower = .75;
    double highGain = 0.01;
    ElapsedTime capBallOpen;
    boolean openCap = false;

    double midPower = 0.5;
    double midGain = 0.03;

    double lowPower = 0.3;
    double lowGain = 0.05;

    double allPower = 1;
    double allPowerGain = 0.01;
    double whiteLightTrigger = .3;
    double encoderTimeOut = 3000.0;

    //State Machines
    enum driveStateMachine{
        slideState, pivotRobot, controllerState, vuforiaMoveToPosition
    }

    //autonomous sequences
    Object[][] sequenceArray = {
            {driveStateMachine.controllerState}
    };
    Object[][] pivot0 = {
            {driveStateMachine.pivotRobot, 0.0, 0.01, .25, highPower, lowPower/2},
            {driveStateMachine.controllerState}
    };
    Object[][] pivot90 = {
            {driveStateMachine.pivotRobot, 90.0, 0.01, .25, highPower, lowPower/2},
            {driveStateMachine.controllerState}
    };
    Object[][] pivot180 = {
            {driveStateMachine.pivotRobot, 180.0, 0.01, .25, highPower, lowPower/2},
            {driveStateMachine.controllerState}
    };
    Object[][] pivot270 = {
            {driveStateMachine.pivotRobot, 270.0, 0.01, .25, highPower, lowPower/2},
            {driveStateMachine.controllerState}
    };
    Object [][] firstBeacon = {
            {driveStateMachine.pivotRobot, 0.0, 0.01, .25, highPower, lowPower/2},
            {driveStateMachine.slideState, 2, 90, 0.0, midGain, allPower, 25.0, 0, midPower, 0.01, null},
            {driveStateMachine.slideState, 4, 0, 0.0, lowGain, lowPower, whiteLightTrigger, 1, null, null, encoderTimeOut},
            {driveStateMachine.slideState, 2, 90, 0.0, allPowerGain, allPower, 6.0, 0, allPower, 0.01, null},
            {driveStateMachine.controllerState}
    };
    Object [][] beacon90Degrees = {
            {driveStateMachine.pivotRobot, -90.0, 0.01, .25, highPower, lowPower/3},
            {driveStateMachine.slideState, 2, 90, 270.0, midGain, allPower, 25.0, 0, midPower, 0.01, null},
            {driveStateMachine.slideState, 4, 0, 270.0, lowGain, lowPower, whiteLightTrigger, 1, null, null, encoderTimeOut},
            {driveStateMachine.slideState, 2, 90, 270.0, allPowerGain, allPower, 6.0, 0, allPower, 0.01, null},
            {driveStateMachine.controllerState}

    };
    Object [][] beacon180 = {
            {driveStateMachine.pivotRobot, 180.0, 0.01, .25, highPower, lowPower/2},
            {driveStateMachine.slideState, 2, 90, 180.0, midGain, allPower, 25.0, 0, midPower, 0.01, null},
            {driveStateMachine.slideState, 4, 180, 180.0, lowGain, lowPower, whiteLightTrigger, 1, null, null, encoderTimeOut},
            {driveStateMachine.slideState, 4, 0, 180.0, lowGain, lowPower, whiteLightTrigger, 1, null, null, encoderTimeOut},
            {driveStateMachine.slideState, 2, 90, 180.0, allPowerGain, allPower, 6.0, 0, allPower, 0.01, null},
            {driveStateMachine.controllerState}
    };
    Object[][] vuforiaPosition = {
            {driveStateMachine.vuforiaMoveToPosition},
            {driveStateMachine.controllerState},
    };

    //data storage and debugging
    boolean printTelemetry = true;
    File gyroFile;

    //sensor reading variables
    double encoderAverage;
    double ultrasonicDistance;
    double stateTime;
    double lightValue;

    //time out variable
    int missCount = 0;


    /*
    Method that averages all of the encoders no matter what direction the robot is traveling
     */

    @Override
    public void runOpMode() throws InterruptedException {
        //Get all of the DC motors from the hardware map
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        shooter = hardwareMap.dcMotor.get("shooter");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        led = hardwareMap.dcMotor.get("led");

        //Get some sensors
        light = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "light");
        ultrasonic = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasonic");
        colorSensor = hardwareMap.colorSensor.get("color");
        colorSensor.enableLed(true);
        //get modules
        deviceModule1 = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        deviceModule2 = hardwareMap.deviceInterfaceModule.get("Device Interface Module 2");
        servoController = hardwareMap.servoController.get("Servo Controller 1");
        //set up LEDS

        //Get beacons
        shooterGate = hardwareMap.servo.get("shooter_gate");
        flag = hardwareMap.servo.get("flag");

        cap_top_limit = hardwareMap.digitalChannel.get("cap_top_limit");
        cap_top_limit.setMode(DigitalChannelController.Mode.INPUT);

        //determine DC Motor modes
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooter.setMaxSpeed(shooterMaxSpeed);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeper.setDirection(DcMotorSimple.Direction.REVERSE);

        //set servo positions
        shooterGate.setPosition(.5);
        flag.setPosition(0);

        //print telemetry
        telemetry.addData("Init", "hardware Initalized");
        telemetry.update();

        //set up elapsed times
        shooterTime = new ElapsedTime();
        shooterToggleTimer = new ElapsedTime();
        harvestToggleTimer = new ElapsedTime();
        shooterSpeedTimer = new ElapsedTime();
        gameTime = new ElapsedTime();
        timer = new ElapsedTime();
        capBallOpen = new ElapsedTime();
        ElapsedTime flagTime = new ElapsedTime();
        ElapsedTime capBallLimitTime = new ElapsedTime();
        telemetry.addData("Init", "elapsed times");
        telemetry.update();
        //turn off toggles
        shooterToggle = false;
        harvestToggle = false;
        //Determine which joystick and whether to use the IMU
        String hand = "right_noIMU";

        boolean allianceSelected = false;
        String alliance = "red";
//        while(!allianceSelected){
//            telemetry.addData("Blue Alliance", "Gamepad 1 Up");
//            telemetry.addData("Red Alliance", "Gamepad 1 Down");
//            if(gamepad1.dpad_up){
//                alliance = "blue";
//                allianceSelected = true;
//            }else if(gamepad1.dpad_down){
//                alliance = "red";
//                allianceSelected = true;
//            }
//            telemetry.update();
//        }
        telemetry.addData("Alliance", alliance);
        telemetry.update();

        while(gamepad1.dpad_down||gamepad1.dpad_up){

        }
//        while (pressed) {
//            telemetry.addData("handedness", "Press dpad left for lefty or dpad right for righty. Press Up for NO IMU Right, and down for NO IMU left");
//            telemetry.update();
//            if (gamepad1.dpad_left) {
//                hand = "left";
//                telemetry.addData("handedness", "lefty");
//                telemetry.update();
//                pressed = false;
//
//            } else if (gamepad1.dpad_right) {
//                hand = "right";
//                telemetry.addData("handedness", "righty");
//                telemetry.update();
//                pressed = false;
//
//            } else if (gamepad1.dpad_up) {
//                hand = "right_noIMU";
//                telemetry.addData("hand", "right no imu");
//                telemetry.update();
//                pressed = false;
//
//            } else if (gamepad1.dpad_down) {
//                hand = "left_noIMU";
//                telemetry.addData("hand", "left no imu");
//                telemetry.update();
//                pressed = false;
//
//            }
//        }

       /* //configure gyro sensor depending on variable
        if (hand.equals("left") || hand.equals("right")) {
            if (MecanumDrive.sensor == 0) {
                gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
                gyro.calibrate();
                while (gyro.isCalibrating()) {

                }
            } else if (MecanumDrive.sensor == 2) {
                //navx device setup
                navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("Device Interface Module 1"),
                        0,
                        AHRS.DeviceDataType.kProcessedData);

                while (navx_device.isCalibrating()) {

                }
            }

        }*/

        telemetry.addData("Init", "Gyro sensor or IMU calibrated");
        telemetry.update();
        if (hand.equals("right_noIMU") || hand.equals("left_noIMU")) {
            offset = 0;
        } else {
            String gyroFileName = "lastGyroValue.txt";
            gyroFile = AppUtil.getInstance().getSettingsFile(gyroFileName);
            String s = ReadWriteFile.readFile(gyroFile);
            offset = Double.parseDouble(s);
        }
        telemetry.addData("Init", "Offset set");
        telemetry.update();


        int seqCounter = 0;
        drive = new MecanumDrive(rf, rb, lf, lb, gyro, telemetry);
        telemetry.addData("init", "done");
        //telemetry.addData("hand", hand);
        telemetry.update();


        waitForStart();
        //beacons.activate();

        if (hand.equals("left") || hand.equals("right")) {
            if (MecanumDrive.sensor == 0 && (hand.equals("left") || hand.equals("right"))) {
                gyro.resetZAxisIntegrator();
            } else if (MecanumDrive.sensor == 2 && (hand.equals("left") || hand.equals("right"))) {
                //navx_device.zeroYaw();
            }
        }
        telemetry.addData("Init", "Sensor Calibration");
        telemetry.update();
        //start timer
        gameTime.reset();
        telemetry.addData("Init", "Game Time Reset");
        telemetry.update();
        int redColor, blueColor;
        //start up lights
        telemetry.addData("Init", "LED init");
        telemetry.update();

        while(opModeIsActive()){
            redColor = colorSensor.red();
            blueColor = colorSensor.blue();


            telemetry.addData("Game Time", 120 - gameTime.seconds());
            //Get sensor values
            stateTime = timer.seconds();
            double ultrasonicTemp = ultrasonic.cmUltrasonic();
            if(ultrasonicTemp < 100) {
                ultrasonicDistance = ultrasonicTemp;
            }
            lightValue = light.getLightDetected();
            encoderAverage = drive.averageEncoders();
            //get Gyro value only if using it for frame of reference

            if(MecanumDrive.sensor==0&&(hand.equals("right") || hand.equals("left"))){
                IMUAngle = -gyro.getHeading();
            }else if(MecanumDrive.sensor==2&&(hand.equals("right") || hand.equals("left"))){
                //IMUAngle = navx_device.getYaw();
            }
            IMUAngle = (IMUAngle + offset)%360;
            //write to file if using IMU

            if(hand.equals("right") || hand.equals("left")) {
                ReadWriteFile.writeFile(gyroFile, String.valueOf(IMUAngle));
            }


            //Robot motion control
            switch((driveStateMachine)sequenceArray[seqCounter][0]){
                /*
                This state is for using the controllers
                without the software taking away control
                 */
                case controllerState:
                    timer.reset();

                    if(hand.equals("left")||hand.equals("right")){
                        if(gamepad1.dpad_up){
                            seqCounter = 0;
                            sequenceArray = pivot90;
                        }else if(gamepad1.dpad_down){
                            seqCounter = 0;
                            sequenceArray = pivot270;
                        }else if(gamepad1.dpad_left){
                            seqCounter = 0;
                            sequenceArray = pivot0;
                        }else if(gamepad1.dpad_right) {
                            seqCounter = 0;
                            sequenceArray = pivot180;
                        }
                        else if(hand.equals("left")){
                            angle = drive.joystickToAngle(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                            speed = drive.returnRadius(gamepad1.left_stick_x, -gamepad1.left_stick_y);

                            pivotSpeed = -gamepad1.right_stick_x;

                        }else if(hand.equals("right")){
                            angle = drive.joystickToAngle(gamepad1.right_stick_x, -gamepad1.right_stick_y);
                            speed = drive.returnRadius(gamepad1.right_stick_x, -gamepad1.right_stick_y);

                            pivotSpeed = -gamepad1.left_stick_x;
                        }
                    }else if(hand.equals("right_noIMU")){
                        angle = drive.joystickToAngle(gamepad1.right_stick_x, -gamepad1.right_stick_y);
                        speed = drive.returnRadius(gamepad1.right_stick_x, -gamepad1.right_stick_y);
                        IMUAngle = 0;
                        offset = 0;
                        pivotSpeed = -gamepad1.left_stick_x;
                        offset = 0;
                        IMUAngle = 0;
                    }
                    else if(hand.equals("left_noIMU")) {
                        angle = drive.joystickToAngle(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                        speed = drive.returnRadius(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                        IMUAngle = 0;
                        offset = 0;
                        pivotSpeed = -gamepad1.right_stick_x;
                        offset = 0;
                        IMUAngle = 0;
                    }

                    angle -= IMUAngle;
                    angle %= 360;

                    //telemetry.addData("Angle: ", angle);
                    drive.pivotSlide(angle, speed, true, pivotSpeed);

                    break;
                /*
                This state is for sliding in a certain direction until a sensor value
                 */
                case slideState:
                    if(gamepad1.b){
                        seqCounter++;
                        drive.setPowerAll(0, 0, 0, 0);
                    }else {
                        switch ((int) sequenceArray[seqCounter][1]) {

                            case 1: //goes while encoder count is less than desired. next parameter is the distance in inches.
                                double encoderError = ((double) sequenceArray[seqCounter][6] * countsPerInch) - encoderAverage; //encoderError = target encoder distance - current encoder position
                                proportionalPower = (encoderError * (double) sequenceArray[seqCounter][9]) + (double) sequenceArray[seqCounter][8]; // Speed = error*gain + minSpeed

                                if (proportionalPower > (double) sequenceArray[seqCounter][5]) { //If the speed is greater than the maxSpeed, set the speed to the max speed
                                    proportionalPower = (double) sequenceArray[seqCounter][5];
                                } else if (proportionalPower < (double) sequenceArray[seqCounter][8]) { //If the speed is less than the minSpeed, set the speed to the min speed
                                    proportionalPower = (double) sequenceArray[seqCounter][8];
                                }

                                if (encoderError > 0) { //Move the robot while the encoderError is greater than 0
                                    drive.slideAngleIMU(((int) sequenceArray[seqCounter][2]), proportionalPower, true, ((double)sequenceArray[seqCounter][3] - offset)%360, (double) sequenceArray[seqCounter][4]);
                                } else {
                                    //determine brake vs. coast
                                    if ((int) sequenceArray[seqCounter][7] == 1) {
                                        drive.setPowerAll(0, 0, 0, 0);
                                        drive.resetEncoders();
                                    }
                                    drive.softResetEncoder();
                                    timer.reset();
                                    seqCounter++;
                                }
                                if (printTelemetry) {
                                    telemetry.addData("State", "Slide State");
                                    telemetry.addData("Inches", encoderAverage / countsPerInch);
                                }
                                break;
                            case 2: //goes while US distance is greater than desired
                                //Parameters: Case, Angle, orientation, angle gain, max power, desired distance, brake or coast, min power, power gain
                                distanceError = ultrasonicDistance - (double) sequenceArray[seqCounter][6]; //error = current distance - desired distance
                                proportionalPower = (double) sequenceArray[seqCounter][8] + (distanceError * (double) sequenceArray[seqCounter][9]); //power to minPower + (error * gain)
                                if (proportionalPower > (double) sequenceArray[seqCounter][5]) { //if the power is greater than the maxPower, set the power to the maxPower
                                    proportionalPower = (double) sequenceArray[seqCounter][5];
                                } else if (proportionalPower < (double) sequenceArray[seqCounter][8]) { // If the power is less than the minPower, set the power to the minPower
                                    proportionalPower = (double) sequenceArray[seqCounter][8];
                                }

                                if (distanceError > 0) { //Move the robot while the calculated error is greater than 0. It will stop once the distance has been reached
                                    drive.slideAngleIMU((((Integer) sequenceArray[seqCounter][2]).doubleValue()), proportionalPower, true, ((double) sequenceArray[seqCounter][3]-offset)%360, (double) sequenceArray[seqCounter][4]);
                                } else {
                                    if ((int) sequenceArray[seqCounter][7] == 1) {
                                        drive.setPowerAll(0, 0, 0, 0);
                                        drive.resetEncoders();
                                    }
                                    drive.softResetEncoder();
                                    timer.reset();
                                    seqCounter++;
                                }
                                if (printTelemetry) {
                                    telemetry.addData("current distance", ultrasonicDistance);
                                    telemetry.addData("error", distanceError);
                                    telemetry.addData("state", "ultrasonic");
                                }

                                break;

                            case 3: //goes while US distance is less than desired
                                //Parameters: Case, Angle, orientation, angle gain, max power, desired distance, brake or coast, min power, power gain
                                distanceError = -(ultrasonicDistance - (double) sequenceArray[seqCounter][6]); //error = current distance - desired distance
                                proportionalPower = (double) sequenceArray[seqCounter][8] + (distanceError * (double) sequenceArray[seqCounter][9]); //power to minPower + (error * gain)
                                if (proportionalPower > (double) sequenceArray[seqCounter][5]) { //if the power is greater than the maxPower, set the power to the maxPower
                                    proportionalPower = (double) sequenceArray[seqCounter][5];
                                } else if (proportionalPower < (double) sequenceArray[seqCounter][8]) { // If the power is less than the minPower, set the power to the minPower
                                    proportionalPower = (double) sequenceArray[seqCounter][8];
                                }

                                if (distanceError > 0) { //Move the robot while the calculated error is greater than 0. It will stop once the distance has been reached
                                    drive.slideAngleIMU((((Integer) sequenceArray[seqCounter][2]).doubleValue()), proportionalPower, true, ((double) sequenceArray[seqCounter][3] - offset)%360, (double) sequenceArray[seqCounter][4]);
                                } else {
                                    if ((int) sequenceArray[seqCounter][7] == 1) {
                                        drive.setPowerAll(0, 0, 0, 0);
                                        drive.resetEncoders();
                                    }
                                    drive.softResetEncoder();
                                    timer.reset();
                                    seqCounter++;
                                }
                                if (printTelemetry) {
                                    telemetry.addData("current distance", ultrasonicDistance);
                                    telemetry.addData("error", distanceError);
                                    telemetry.addData("state", "ultrasonic");
                                }
                                break;

                            case 4: //goes while light value is less than desired
                                if (lightValue < (double) sequenceArray[seqCounter][6]) {
                                    drive.slideAngleIMU((((Integer) sequenceArray[seqCounter][2]).doubleValue()), (double) sequenceArray[seqCounter][5], true, ((double) sequenceArray[seqCounter][3]-offset)%360, (double) sequenceArray[seqCounter][4]);
                                } else {
                                    if ((int) sequenceArray[seqCounter][7] == 1) {
                                        drive.setPowerAll(0, 0, 0, 0);
                                        drive.resetEncoders();
                                    }
                                    timer.reset();
                                    drive.softResetEncoder();
                                    seqCounter++;
                                }
                                if (printTelemetry) {
                                    telemetry.addData("State", "slide light");
                                    telemetry.addData("light value", lightValue);
                                }
                                break;

                            case 5: //goes while timer is less than desired
                                if ((double) stateTime < (double) sequenceArray[seqCounter][6]) {
                                    drive.slideAngleIMU((((Integer) sequenceArray[seqCounter][2]).doubleValue()), (double) sequenceArray[seqCounter][5], true, ((double) sequenceArray[seqCounter][3] - offset)%360, (double) sequenceArray[seqCounter][4]);
                                } else {
                                    if ((int) sequenceArray[seqCounter][7] == 1) {
                                        drive.setPowerAll(0, 0, 0, 0);
                                        drive.resetEncoders();
                                    }
                                    timer.reset();
                                    drive.softResetEncoder();
                                    seqCounter++;
                                }
                                if (printTelemetry) {
                                    telemetry.addData("state", "time ");
                                    telemetry.addData("timer", stateTime);
                                }
                                break;


                            case 7: //Moves only if we have a color detected

                                //DEPRECIATED
                            /*
                            if (rightBeacon.equals("red")|| rightBeacon.equals("blue, red")) {
                                if (drive.slideAngleIMU(((Integer) sequenceArray[seqCounter][1]).doubleValue(), (double) sequenceArray[seqCounter][2], ultrasonic.cmUltrasonic() > (double) sequenceArray[seqCounter][4], (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]) == 1) {
                                    telemetry.addData("state", ultrasonic.cmUltrasonic());
                                } else {
                                    timer.reset();
                                    drive.softResetEncoder();
                                    seqCounter++;
                                }
                            } else {
                                timer.reset();
                                drive.softResetEncoder();
                                seqCounter++;
                            }*/
                                break;
                        }
                    }
                    break;
                /*
                This case is for pivoting to a certain angle
                 */
                case pivotRobot: //Pivots robot to a desired angle
                    // pivot to angle for a certain time
                    if(gamepad1.b){
                        drive.setPowerAll(0, 0, 0, 0);
                        seqCounter = sequenceArray.length-1;
                    }else{
                        if((IMUAngle>(double) sequenceArray[seqCounter][1]-5&&IMUAngle<(double) sequenceArray[seqCounter][1]+5)||reachedPivot){
                            reachedPivot = true;
                            //telemetry.addData("Angle range", true);
                        }else{
                            timer.reset();
                        }
                        if (timer.seconds()<(double) sequenceArray[seqCounter][3]){
                            drive.pivotToAngleIMU((double) sequenceArray[seqCounter][1]-offset, (double) sequenceArray[seqCounter][2], true, (double) sequenceArray[seqCounter][4],(double) sequenceArray[seqCounter][5]);
                        } else {
                            timer.reset();
                            drive.resetEncoders();
                            drive.softResetEncoder();
                            reachedPivot = false;
                            seqCounter++;
                        }
                    }

                    break;
            }

            /*
            Turns the shooter on or off
             */

            shooter.setPower(shootingSpeed);


            /*
            Opens the shooter gate
             */
            if(gamepad1.right_trigger>.2){
                shooterGate.setPosition(0);

            }else{
                shooterGate.setPosition(.5);
            }


            if(gamepad1.right_bumper == true){
                sweeper.setPower(1);
            }
            else if(gamepad1.left_bumper == true){
                sweeper.setPower(0);
            }
            /*
            Sweeps out to reject particles
             */

            /*
            cap ball servo positions
             */
            /*
            if(gamepad2.dpad_left){
                RT_LB_CapBall.setPosition(rightCapBallOpenPosition);
                LT_RB_CapBall.setPosition(leftCapBallOpenPosition);
                capBallOpen.reset();
            }else if(gamepad2.dpad_right){
                LT_RB_CapBall.setPosition(rightBottomCapBallStart);
                RT_LB_CapBall.setPosition(leftBottomCapBallStart);
            }else if(gamepad2.dpad_up){
                LT_RB_CapBall.setPosition(0.5);
                RT_LB_CapBall.setPosition(0.5);
            }
*/
            /*
            cap ball lift and lower
             */
            /*
            if(gamepad2.right_trigger > 0.2 && cap_top_limit.getState() == false){
                capBall.setPower(capBallLiftSpeed);
            }else if(cap_top_limit.getState() == true && gamepad2.right_trigger > 0.2){
                telemetry.addData("Cap Ball", "Limit Reached");
                capBall.setPower(capBallRetractSpeed);
            }
            else if(gamepad2.left_trigger > 0.2) {
                capBall.setPower(capBallRetractSpeed);
            }
            else{
                capBall.setPower(0);
            }*/
            /*
            Turn off frame of reference through the match
             */


            /*
            calculate shooter speed
             */
            if(shooterSpeedTimer.seconds()>.5){
                previousEncoder = currentEncoder;
                currentEncoder = shooter.getCurrentPosition();
                shooterSpeed = (currentEncoder-previousEncoder)/shooterSpeedTimer.seconds();
                shooterSpeedTimer.reset();
            }
            /*
            driver feedback for wrong particle color detection
             */
            if(alliance.equals("red") && blueColor > 5){
                flagTime.reset();
                telemetry.addData("Wrong Particle Swept","");
                telemetry.update();
                flag.setPosition(0.5);
            }else if(alliance.equals("blue") && redColor > 5){
                flagTime.reset();
                telemetry.addData("Wrong Particle Swept","");
                telemetry.update();
                flag.setPosition(0.5);
            }else{
                if(flagTime.seconds() < 3){
                    telemetry.addData("Wrong Particle Swept","");
                    telemetry.update();
                }
                else{
                    flag.setPosition(0);
                    telemetry.update();
                }
            }

            /*
            give driver feedback for shooter speed
             */
            if(shooterSpeed<shooterHighRange&&shooterSpeed>shooterLowRange){
                deviceModule1.setLED(0, true);
                deviceModule1.setLED(1, false);
                deviceModule2.setLED(0, true);
                deviceModule2.setLED(1, false);
                //led.setPower(1);
                telemetry.addData("Shooter", "Up to speed");
            }else{
                deviceModule1.setLED(0, false);
                deviceModule1.setLED(1, false);
                deviceModule2.setLED(0, false);
                deviceModule2.setLED(1, false);
                //led.setPower(0);
                telemetry.addData("Shooter", "not up to speed");
            }
            led.setPower(0);
            //telemetry.addData("State", (driveStateMachine)sequenceArray[seqCounter][0]);
            //telemetry.addData("Seq Counter", seqCounter);
            telemetry.update();
        }

        /*
        Turn off everything
         */
        if(MecanumDrive.sensor == 2 && (hand.equals("left")||hand.equals("right"))){
            //navx_device.close();
        }
        // telemetry.addData("Stop", "NavX Closed");
        telemetry.update();
        drive.setPowerAll(0, 0, 0, 0);
        shooter.setPower(0);
        sweeper.setPower(0);
        telemetry.addData("Stop", "Motor Psower = 0");
        telemetry.update();
        telemetry.addData("Stop", "LED Color Set");
        telemetry.addData("Stop", "Done");
        telemetry.update();
        stop();

    }
}

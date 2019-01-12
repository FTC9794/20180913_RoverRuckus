package org.firstinspires.ftc.teamcode.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.omnidirectional.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.sampling.GoldMineralDetector;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.ITeamMarker;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.ServoArmDrop;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by Sarthak on 10/26/2018.
 */
@Autonomous(name = "\uD83E\uDD16 Depot Autonomous", group = "Autonomous")
public class RoverRuckusDepotAutonomousProgram extends LinearOpMode {

    IDrivetrain drive;
    DcMotor right_front, right_back, left_front, left_back;
    DcMotor verticalLeft, verticalRight, horizontal, horizontal2;
    DcMotor mineral_rotation, mineralExtension;
    DcMotor intakeRotation;
    ArrayList motors, encoders;

    ITeamMarker teamMarker;
    Servo team_marker;

    Servo scanner;
    double rightPosition = 0.7;
    double leftPosition = 0.3;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamNameLeft;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

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
    private GoldMineralDetector detector = null;

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

    double motorPowerRampDownStartPosition = 10 * COUNTS_PER_INCH;
    double motorPowerRampDownEndPosition = 5 * COUNTS_PER_INCH;

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

    File mineralExtensionEncoderPosition = AppUtil.getInstance().getSettingsFile("mineralExtensionEncoderPosition.txt");
    File mineralRotationEncoderPosition = AppUtil.getInstance().getSettingsFile("mineralRotationEncoderPosition.txt");
    File intakeRotationEncoderPosition = AppUtil.getInstance().getSettingsFile("intakeRotationEncoderPosition.txt");

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

        webcamNameLeft = hardwareMap.get(WebcamName.class, "Webcam 1");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        parameters.fillCameraMonitorViewParent = true;

        parameters.cameraName = webcamNameLeft;

        telemetry.addData("Init", "Webcam and License Key Set");
        telemetry.update();

        vuforia = new Dogeforia(parameters);
        telemetry.addData("Init", "Dogeforia Params Set");
        telemetry.update();

        vuforia.enableConvertFrameToBitmap();

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        telemetry.addData("Init", "Set Trackables Names");
        telemetry.update();

        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        allTrackables.addAll(targetsRoverRuckus);
        telemetry.addData("Init", "Added Trackables");
        telemetry.update();

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        telemetry.addData("Init", "Set Target Locations");
        telemetry.update();

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

        detector = new GoldMineralDetector();
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;

        telemetry.addData("Init", "Set up detector");
        telemetry.update();

        vuforia.setDogeCVDetector(detector);
        vuforia.showDebug();
        vuforia.start();

        telemetry.addData("Init", "Complete");
        telemetry.update();

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
        //Release Hang Latch
        hang_latch.setPosition(1);
        waitMilliseconds(750, runtime);

        //Delatch from hanger
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineral_rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setTargetPosition(4900);
        hang.setPower(1);
        mineral_rotation.setTargetPosition(170);
        mineral_rotation.setPower(1);
        intakeRotation.setTargetPosition(175);
        intakeRotation.setPower(1);

        runtime.reset();
        while(hang.isBusy() && opModeIsActive()){
            if(!mineral_rotation.isBusy()){
                if(rotation_limit.getState()){
                    mineral_rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    mineral_rotation.setPower(-0.15);
                }else{
                    mineral_rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    mineral_rotation.setPower(0);
                }
            }
            telemetry.addData("hang pos", hang.getCurrentPosition());
            telemetry.update();
        }
        hang.setPower(0);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vuforia.enableDogeCV();
        waitMilliseconds(750, runtime);

        //Scan for mineral
        globalCoordinatePositionUpdate();
        boolean found = detector.isFound();
        if(found && detector.getScreenPosition().y > 140){
            mineralLocation = location.CENTER;
        }else if (found && detector.getScreenPosition().x > 450 && detector.getScreenPosition().y > 140) {
            mineralLocation = location.RIGHT;
        }
        else{
            while(opModeIsActive() && scanner.getPosition() > leftPosition){
                scanner.setPosition(scanner.getPosition() - 0.001);
                telemetry.addData("Y Position", detector.getScreenPosition().y);
                telemetry.update();
            }
            runtime.reset();
            while(runtime.milliseconds() < 1000 && opModeIsActive()){
                telemetry.addData("Y Position", detector.getScreenPosition().y);
                telemetry.update();
            }

            found = detector.isFound();
            if(found && detector.getScreenPosition().y > 140) {
                mineralLocation = location.LEFT;
            }else {
                mineralLocation = location.RIGHT;
            }
        }

        vuforia.disableDogeCV();
        scanner.setPosition(0.5);
        team_marker.setPosition(0.5);

        ReadWriteFile.writeFile(mineralExtensionEncoderPosition, String.valueOf(mineralExtension.getCurrentPosition()));
        ReadWriteFile.writeFile(mineralRotationEncoderPosition, String.valueOf(mineral_rotation.getCurrentPosition()));
        ReadWriteFile.writeFile(intakeRotationEncoderPosition, String.valueOf(intakeRotation.getCurrentPosition()));

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

        ReadWriteFile.writeFile(mineralExtensionEncoderPosition, String.valueOf(mineralExtension.getCurrentPosition()));
        ReadWriteFile.writeFile(mineralRotationEncoderPosition, String.valueOf(mineral_rotation.getCurrentPosition()));
        ReadWriteFile.writeFile(intakeRotationEncoderPosition, String.valueOf(intakeRotation.getCurrentPosition()));

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

        intakeRotation.setTargetPosition(10);
        intakeRotation.setPower(1);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setTargetPosition(2950);
        hang.setPower(0.5);


        //Drive to alliance depot
        drive.softResetEncoder();
        while(opModeIsActive() && drive.move(drive.getEncoderDistance(), 46*COUNTS_PER_INCH, 10*COUNTS_PER_INCH,
                0, 30*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, -50 , DEFAULT_PID, 135
                ,0.5*COUNTS_PER_INCH, 0));
        drive.stop();

        //Drop team marker
        teamMarker.drop();
        waitMilliseconds(500, runtime);

        ReadWriteFile.writeFile(mineralExtensionEncoderPosition, String.valueOf(mineralExtension.getCurrentPosition()));
        ReadWriteFile.writeFile(mineralRotationEncoderPosition, String.valueOf(mineral_rotation.getCurrentPosition()));
        ReadWriteFile.writeFile(intakeRotationEncoderPosition, String.valueOf(intakeRotation.getCurrentPosition()));

        //Drive to crater to park
        drive.softResetEncoder();
        while(opModeIsActive() && drive.move(drive.getEncoderDistance(), 55*COUNTS_PER_INCH, 25*COUNTS_PER_INCH,
                0, 55*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 138 , DEFAULT_PID, 135
                ,0.5*COUNTS_PER_INCH, 0)){
            if(drive.getEncoderDistance() > 25 * COUNTS_PER_INCH){
                teamMarker.hold();
            }
        }
        drive.stop();

        mineralExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineralExtension.setTargetPosition(1000);
        mineralExtension.setPower(1);

        while (opModeIsActive()){
            ReadWriteFile.writeFile(mineralExtensionEncoderPosition, String.valueOf(mineralExtension.getCurrentPosition()));
            ReadWriteFile.writeFile(mineralRotationEncoderPosition, String.valueOf(mineral_rotation.getCurrentPosition()));
            ReadWriteFile.writeFile(intakeRotationEncoderPosition, String.valueOf(intakeRotation.getCurrentPosition()));
            drive.stop();
            globalCoordinatePositionUpdate();
            telemetry.addData("Status", "Program Finished");
            telemetry.addData("X Position", robotGlobalXPosition /COUNTS_PER_INCH);
            telemetry.addData("Y Position", robotGlobalYPosition /COUNTS_PER_INCH);
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

        mineralExtension = hardwareMap.dcMotor.get("mineral_extension");
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRotation = hardwareMap.dcMotor.get("intake_rotation");
        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        double distance = distanceFormula(xDistance, yDistance);

        double robotOrientationDifference = targetOrientation - imu.getZAngle();

        //double power = maxPower;
        double power;
        if(distance > motorPowerRampDownStartPosition){
            power = maxPower;
        }else if (distance < motorPowerRampDownEndPosition){
            power = minPower;
        }else{
            double motorRampDownPositionDifference = motorPowerRampDownStartPosition - motorPowerRampDownEndPosition;
            double distanceRampDownDifference = distance - motorPowerRampDownEndPosition;
            power = -((minPower-maxPower)/(motorRampDownPositionDifference))*(distanceRampDownDifference) + minPower;
        }

        double robotMoveAngle;
        robotMoveAngle = Math.toDegrees(Math.atan(xDistance/yDistance));
        if((xDistance < 0 && yDistance < 0) || (xDistance > 0 && yDistance < 0)){
            robotMoveAngle += 180;
        }
        robotMoveAngle = (robotMoveAngle % 360);

        /*dataLogger.addField((float) robotGlobalXPosition);
        dataLogger.addField((float) robotGlobalYPosition);
        dataLogger.addField((float) xDistance);
        dataLogger.addField((float) yDistance);
        dataLogger.addField((float) power);
        dataLogger.addField((float) imu.getZAngle());
        dataLogger.addField((float) robotOrientationDifference);
        dataLogger.addField((float) robotMoveAngle);
        dataLogger.addField((float) (robotMoveAngle - imu.getZAngle()));
        dataLogger.newLine();*/

        if(!(Math.abs(yDistance) < 0.75 * COUNTS_PER_INCH && Math.abs(xDistance) < 0.75 * COUNTS_PER_INCH
                && Math.abs(robotOrientationDifference) < 5)){
            drive.move(0, distance, distance, 0, distance, power, power,
                    robotMoveAngle, DEFAULT_PID, targetOrientation, DEFAULT_ERROR_DISTANCE, 500);
            telemetry.addData("Encoder Distance", distance/COUNTS_PER_INCH);
            telemetry.addData("X Distance", xDistance/COUNTS_PER_INCH);
            telemetry.addData("Y Distance", yDistance/COUNTS_PER_INCH);
            telemetry.addData("Move Angle", robotMoveAngle);

            return true;
        }else{
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

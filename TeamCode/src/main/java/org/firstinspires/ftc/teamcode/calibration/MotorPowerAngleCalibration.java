package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataLogger;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.omnidirectional.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

import java.util.ArrayList;
import java.util.Date;

/**
 * Created by Sarthak on 3/10/2019.
 */
@TeleOp(name = "Motor Power & Angle Calibration", group = "Test")
public class MotorPowerAngleCalibration extends LinearOpMode {
    IDrivetrain drive;
    DcMotor right_front, right_back, left_front, left_back;
    DcMotor verticalLeft, verticalRight, horizontal, horizontal2;
    ArrayList motors, encoders;

    IIMU imu;
    BNO055IMU boschIMU;

    DataLogger data;
    Date date;

    //Drivetrain Movement Constants
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
    final double normalEncoderWheelPositionAngleFromRotationAxis = -20.63;

    double motorPowerRampDownStartPosition = 30 * COUNTS_PER_INCH;
    double motorPowerRampDownEndPosition = 30 * COUNTS_PER_INCH;

    double changeInRobotOrientation = 0;

    final double constantMaxPower = 0.75;
    final double[] variableMotorPowers = {0.75, 0.6, 0.45, 0.3, 0.15, 0.075, 0, -0.075, -0.15, -0.3, -0.45,-0.6, -0.75};

    double previousXRecordedPosition = 0;
    double previousYRecordedPosition = 0;

    ElapsedTime runTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize data logger
        data = new DataLogger("Drivetrain Motor Power Calibration");
        data.addField("angle");
        data.addField("X Displacement");
        data.addField("Y Displacement");
        data.addField("LF_RB Power");
        data.addField("RF_LB Power");
        data.newLine();

        //Set motor behaviors and hardware map
        setMotorBehaviors();

        //Initialize IMU
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.initialize();
        imu.setOffset(0);
        telemetry.addData("Status", "IMU Instantiated");
        telemetry.update();

        //Initialize Drivetrain
        drive = new MecanumDrive(motors, imu, telemetry, encoders);
        date = new Date();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();
        drive.softResetEncoder();

        //Test first quadrant
        for(int i = 0; i < variableMotorPowers.length; i++){
            //Find the variable motor power to test
            double variableMotorPower = variableMotorPowers[i];

            //Drive 24 inches with one constant power and one variable power
            while(drive.getEncoderDistance() < 24*COUNTS_PER_INCH && opModeIsActive()){
                //globalCoordinatePositionUpdate();

                right_front.setPower(constantMaxPower);
                left_back.setPower(constantMaxPower);

                left_front.setPower(variableMotorPower);
                right_back.setPower(variableMotorPower);
            }
            drive.stop();
            waitMilliseconds(1000, runTime);
            globalCoordinatePositionUpdate();

            //Record the angle based on the x, y position and the motor powers applied
            double x = normalEncoderWheelPosition = horizontal.getCurrentPosition();
            double y1 = verticalLeft.getCurrentPosition();
            double y2 = -verticalRight.getCurrentPosition();

            double y = -((y1 + y2)/2);
            double angle = Math.toDegrees(Math.atan(x/y));
            if((x < 0 && y < 0) || (x > 0 && y < 0)){
                angle += 180;
            }
            angle = (angle % 360);
            double rflbPower = constantMaxPower;
            double lfrbPower = variableMotorPower;

            data.addField((float) angle);
            data.addField((float) rflbPower);
            data.addField((float) lfrbPower);
            data.newLine();

            //Wait for user to confirm measurements and calculations
            while(gamepad1.atRest() && opModeIsActive()){
                telemetry.addData("Recorded Angle", angle);
                telemetry.addData("Recorded X Position", x/COUNTS_PER_INCH);
                telemetry.addData("Recorded Y Position", y/COUNTS_PER_INCH);
                telemetry.addData("RF, LB Power", rflbPower);
                telemetry.addData("LF, RB Power", lfrbPower);
                globalCoordinatePositionUpdate();
                telemetry.update();

                right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                telemetry.addData("Vertical Left Position", verticalLeft.getCurrentPosition());
                telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
                telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());

                drive.stop();
                telemetry.update();
            }

            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            drive.softResetEncoder();
        }

        //Test second quadrant
        for(int i = 0; i < variableMotorPowers.length; i++){
            //Find the variable motor power to test
            double variableMotorPower = variableMotorPowers[i];

            //Drive 24 inches with one constant power and one variable power
            while(drive.getEncoderDistance() < 24*COUNTS_PER_INCH && opModeIsActive()){
                globalCoordinatePositionUpdate();

                right_front.setPower(-constantMaxPower);
                left_back.setPower(-constantMaxPower);

                left_front.setPower(variableMotorPower);
                right_back.setPower(variableMotorPower);
            }
            drive.stop();
            waitMilliseconds(1000, runTime);
            globalCoordinatePositionUpdate();

            //Record the angle based on the x, y position and the motor powers applied
            double x = normalEncoderWheelPosition = horizontal.getCurrentPosition();
            double y1 = verticalLeft.getCurrentPosition();
            double y2 = -verticalRight.getCurrentPosition();

            double y = -((y1 + y2)/2);
            double angle = Math.toDegrees(Math.atan(x/y));
            if((x < 0 && y < 0) || (x > 0 && y < 0)){
                angle += 180;
            }
            angle = (angle % 360);
            double rflbPower = -constantMaxPower;
            double lfrbPower = variableMotorPower;

            data.addField((float) angle);
            data.addField((float) rflbPower);
            data.addField((float) lfrbPower);
            data.newLine();

            //Wait for user to confirm measurements and calculations
            while(gamepad1.atRest() && opModeIsActive()){
                telemetry.addData("Recorded Angle", angle);
                telemetry.addData("Recorded X Position", x/COUNTS_PER_INCH);
                telemetry.addData("Recorded Y Position", y/COUNTS_PER_INCH);
                telemetry.addData("RF, LB Power", rflbPower);
                telemetry.addData("LF, RB Power", lfrbPower);
                globalCoordinatePositionUpdate();
                telemetry.update();

                right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                telemetry.addData("Vertical Left Position", verticalLeft.getCurrentPosition());
                telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
                telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());

                drive.stop();
                telemetry.update();
            }

            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            drive.softResetEncoder();
        }

        //Test third quadrant
        for(int i = 0; i < variableMotorPowers.length; i++){
            //Find the variable motor power to test
            double variableMotorPower = variableMotorPowers[i];

            //Drive 24 inches with one constant power and one variable power
            while(drive.getEncoderDistance() < 24*COUNTS_PER_INCH && opModeIsActive()){
                globalCoordinatePositionUpdate();

                right_front.setPower(variableMotorPower);
                left_back.setPower(variableMotorPower);

                left_front.setPower(constantMaxPower);
                right_back.setPower(constantMaxPower);
            }
            drive.stop();
            waitMilliseconds(1000, runTime);
            globalCoordinatePositionUpdate();

            //Record the angle based on the x, y position and the motor powers applied
            double x = normalEncoderWheelPosition = horizontal.getCurrentPosition();
            double y1 = verticalLeft.getCurrentPosition();
            double y2 = -verticalRight.getCurrentPosition();

            double y = -((y1 + y2)/2);
            double angle = Math.toDegrees(Math.atan(x/y));
            if((x < 0 && y < 0) || (x > 0 && y < 0)){
                angle += 180;
            }
            angle = (angle % 360);
            double rflbPower = variableMotorPower;
            double lfrbPower = constantMaxPower;

            data.addField((float) angle);
            data.addField((float) rflbPower);
            data.addField((float) lfrbPower);
            data.newLine();

            //Wait for user to confirm measurements and calculations
            while(gamepad1.atRest() && opModeIsActive()){
                telemetry.addData("Recorded Angle", angle);
                telemetry.addData("Recorded X Position", x/COUNTS_PER_INCH);
                telemetry.addData("Recorded Y Position", y/COUNTS_PER_INCH);
                telemetry.addData("RF, LB Power", rflbPower);
                telemetry.addData("LF, RB Power", lfrbPower);
                globalCoordinatePositionUpdate();
                telemetry.update();

                right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                telemetry.addData("Vertical Left Position", verticalLeft.getCurrentPosition());
                telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
                telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());

                drive.stop();
                telemetry.update();
            }

            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            drive.softResetEncoder();
        }

        //Test fourth quadrant
        for(int i = 0; i < variableMotorPowers.length; i++){
            //Find the variable motor power to test
            double variableMotorPower = variableMotorPowers[i];

            //Drive 24 inches with one constant power and one variable power
            while(drive.getEncoderDistance() < 24*COUNTS_PER_INCH && opModeIsActive()){
                globalCoordinatePositionUpdate();

                right_front.setPower(variableMotorPower);
                left_back.setPower(variableMotorPower);

                left_front.setPower(-constantMaxPower);
                right_back.setPower(-constantMaxPower);
            }
            drive.stop();
            waitMilliseconds(1000, runTime);
            globalCoordinatePositionUpdate();

            //Record the angle based on the x, y position and the motor powers applied
            double x = normalEncoderWheelPosition = horizontal.getCurrentPosition();
            double y1 = verticalLeft.getCurrentPosition();
            double y2 = -verticalRight.getCurrentPosition();

            double y = -((y1 + y2)/2);
            double angle = Math.toDegrees(Math.atan(x/y));
            if((x < 0 && y < 0) || (x > 0 && y < 0)){
                angle += 180;
            }
            angle = (angle % 360);
            double rflbPower = variableMotorPower;
            double lfrbPower = -constantMaxPower;

            data.addField((float) angle);
            data.addField((float) rflbPower);
            data.addField((float) lfrbPower);
            data.newLine();

            //Wait for user to confirm measurements and calculations
            while(gamepad1.atRest() && opModeIsActive()){
                telemetry.addData("Recorded Angle", angle);
                telemetry.addData("Recorded X Position", x/COUNTS_PER_INCH);
                telemetry.addData("Recorded Y Position", y/COUNTS_PER_INCH);
                telemetry.addData("RF, LB Power", rflbPower);
                telemetry.addData("LF, RB Power", lfrbPower);
                globalCoordinatePositionUpdate();
                telemetry.update();

                right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                telemetry.addData("Vertical Left Position", verticalLeft.getCurrentPosition());
                telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
                telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());

                drive.stop();
                telemetry.update();
            }

            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            drive.softResetEncoder();
        }
        data.closeDataLogger();

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

    private boolean goToPosition(double targetX, double targetY, double targetOrientation, double maxPower, double minPower){

        globalCoordinatePositionUpdate();

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

    public double distanceFormula(double x, double y){
        double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
        return distance;
    }

    private void waitMilliseconds(double milliseconds, ElapsedTime timer){
        //Reset the timer
        timer.reset();
        //Wait until the time inputted has fully elapsed
        while(opModeIsActive() && timer.milliseconds() < milliseconds){
            globalCoordinatePositionUpdate();
        }
    }
}

package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

/**
 * Created by Sarthak on 1/30/2019.
 */
@TeleOp(name = "Sensor Readings Test", group = "Test")
public class SensorTest extends LinearOpMode {
    DcMotor hang, mineralRotation, mineralExtension, intakeRotation;
    DcMotor verticalRight, verticalLeft, horizontal;

    IIMU imu;
    BNO055IMU boschIMU;

    ModernRoboticsI2cRangeSensor leftWallPing, rightWallPing;

    DigitalChannel rotation_limit, hangLimit;

    Rev2mDistanceSensor latch_detector;

    @Override
    public void runOpMode() throws InterruptedException {
        verticalLeft = hardwareMap.dcMotor.get("rf");
        verticalRight = hardwareMap.dcMotor.get("rb");
        horizontal = hardwareMap.dcMotor.get("lf");

        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Encoder Wheels Instantiated");
        telemetry.update();

        hang = hardwareMap.dcMotor.get("hang");

        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mineralRotation = hardwareMap.dcMotor.get("mineral_rotation");
        mineralRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mineralExtension = hardwareMap.dcMotor.get("mineral_extension");
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeRotation = hardwareMap.dcMotor.get("intake_rotation");
        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "DcMotors Instantiated");
        telemetry.update();

        //Initialize IMU
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.initialize();
        imu.setOffset(0);
        telemetry.addData("Status", "IMU Instantiated");
        telemetry.update();

        leftWallPing = (ModernRoboticsI2cRangeSensor) hardwareMap.get("left_us");
        rightWallPing = (ModernRoboticsI2cRangeSensor) hardwareMap.get("right_us");
        telemetry.addData("Status", "Ultrasonic Instantiated");
        telemetry.update();

        rotation_limit = hardwareMap.digitalChannel.get("rotation_limit");
        hangLimit = hardwareMap.digitalChannel.get("hang_limit");
        telemetry.addData("Status", "Digital Channel Sensors Instantiated");
        telemetry.update();

        latch_detector = (Rev2mDistanceSensor) hardwareMap.get("latch_detector");
        telemetry.addData("Status", "Rev Distance Sensor Instantiated");
        telemetry.update();

        telemetry.addData("Status", "Init Complete");
        telemetry.addLine("Sensor Readings Test");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Vertical Right Encoder Wheel Position", verticalRight.getCurrentPosition());
            telemetry.addData("Vertical Left Encoder Wheel Position", verticalLeft.getCurrentPosition());
            telemetry.addData("Horizontal Encoder Wheel Position", horizontal.getCurrentPosition());

            telemetry.addData("Hang Current Position", hang.getCurrentPosition());
            telemetry.addData("Mineral Extension Current Position", mineralExtension.getCurrentPosition());
            telemetry.addData("Mineral Rotation Current Position", mineralRotation.getCurrentPosition());
            telemetry.addData("Mineral Intake Rotation Current Position", intakeRotation.getCurrentPosition());

            telemetry.addData("IMU Angle", imu.getZAngle());

            telemetry.addData("Left Ultrasonic Sensor CM Distance", leftWallPing.cmUltrasonic());
            telemetry.addData("Right Ultrasonic Sensor CM Distance", rightWallPing.cmUltrasonic());

            telemetry.addData("Rotation Limit State", rotation_limit.getState());
            telemetry.addData("Hang Limit State", hangLimit.getState());

            telemetry.addData("Latch Detector Sensor CM Distance", latch_detector.getDistance(DistanceUnit.CM));

            telemetry.update();
        }
    }
}

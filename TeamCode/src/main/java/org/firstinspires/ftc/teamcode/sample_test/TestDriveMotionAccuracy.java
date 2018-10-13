package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.directional.TankDrive2W;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.omnidirectional.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

import java.util.ArrayList;

/**
 * Created by Sarthak on 9/12/2018.
 */
@Autonomous(name = "Test Drive Motion Accuracy", group = "Test")
public class TestDriveMotionAccuracy extends LinearOpMode {

    IDrivetrain drive;
    DcMotor right_front, right_back, left_front, left_back;
    ArrayList motors;

    IIMU imu;
    BNO055IMU boschIMU;

    ElapsedTime timer;

    final double[] DEFAULT_PID = {.05};
    final double COUNTS_PER_INCH = 45;

    @Override
    public void runOpMode() throws InterruptedException {
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new ArrayList<>();
        motors.add(right_front);
        motors.add(right_back);
        motors.add(left_front);
        motors.add(left_back);
        telemetry.addData("Status", "Motor Hardware Initialized");
        telemetry.update();

        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.initialize();
        imu.setOffset(0);
        telemetry.addData("Status", "IMU Instantiated");
        telemetry.update();

        drive = new MecanumDrive(motors, imu, telemetry);
        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        timer = new ElapsedTime();

        //Program starts here
        waitForStart();
        timer.reset();

        while(drive.pivot(45, 22, 0.25, 0.2, 1000, 2.5, Direction.FASTEST)
                && opModeIsActive());

        timer.reset();
        while (timer.milliseconds() < 1000 && opModeIsActive());

        while(drive.pivot(90, 75, 0.25, 0.2, 1000, 2.5, Direction.FASTEST)
                && opModeIsActive());

        timer.reset();
        while (timer.milliseconds() < 1000 && opModeIsActive());

        while(drive.pivot(180, 150, 0.25, 0.2, 1000, 2.5, Direction.FASTEST)
                && opModeIsActive());

        timer.reset();
        while (timer.milliseconds() < 1000 && opModeIsActive());

        while(drive.pivot(270, 220, 0.25, 0.2, 1000, 2.5, Direction.FASTEST)
                && opModeIsActive());

        timer.reset();
        while (timer.milliseconds() < 1000 && opModeIsActive());

        while(drive.pivot(0, 315, 0.25, 0.2, 1000, 2.5, Direction.FASTEST)
                && opModeIsActive());

        timer.reset();
        while (timer.milliseconds() < 1000 && opModeIsActive());

        //End of the program
        while(opModeIsActive());

    }
}

package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.directional.TankDrive2W;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

import java.util.ArrayList;

/**
 * Created by Sarthak on 9/11/2018.
 */
@Autonomous(name = "Test Motor Encoders", group = "Test")
public class TestMotorEncoders extends LinearOpMode {
    IDrivetrain drive;
    DcMotor right, left;
    ArrayList motors;

    IIMU imu;
    BNO055IMU boschIMU;

    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new ArrayList<>();
        motors.add(right);
        motors.add(left);
        telemetry.addData("Status", "Motor Hardware Initialized");
        telemetry.update();

        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.initialize();
        imu.setOffset(0);
        telemetry.addData("Status", "IMU Instantiated");
        telemetry.update();

        drive = new TankDrive2W(motors, imu, telemetry);
        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Encoder Position", drive.getEncoderDistance());
            telemetry.update();
        }
    }
}

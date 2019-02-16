package org.firstinspires.ftc.teamcode.sample_test;

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

import java.util.ArrayList;

/**
 * Created by Sarthak on 2/12/2019.
 */
@Autonomous(name = "Pivot Test", group = "Test")
public class PivotTest extends LinearOpMode {
    IDrivetrain drive;
    DcMotor right_front, right_back, left_front, left_back;
    DcMotor verticalLeft, verticalRight, horizontal, horizontal2;

    ArrayList motors, encoders;

    final double DEFAULT_MAX_POWER = .75;
    final double DEFAULT_MIN_POWER = .35;

    IIMU imu;
    BNO055IMU boschIMU;

    ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        setMotorBehaviors();

        //Initialize IMU
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.initialize();
        imu.setOffset(0);
        telemetry.addData("Status", "IMU Instantiated");
        telemetry.update();

        //Setup Drivetrain Subsystem
        drive = new MecanumDrive(motors, imu, telemetry, encoders);
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            //Pivot to 90 degrees
            while(drive.pivot(90, 45, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 500,
                    5, Direction.FASTEST) && opModeIsActive());
            drive.stop();

            //Wait
            runtime.reset();
            while(opModeIsActive() && runtime.milliseconds() < 500);

            //Pivot to 180 degrees
            while(drive.pivot(180, 135, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 500,
                    5, Direction.FASTEST) && opModeIsActive());
            drive.stop();

            //Wait
            runtime.reset();
            while(opModeIsActive() && runtime.milliseconds() < 500);

            //Pivot to 270 degrees
            while(drive.pivot(270, 225, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 500,
                    5, Direction.FASTEST) && opModeIsActive());
            drive.stop();

            //Wait
            runtime.reset();
            while(opModeIsActive() && runtime.milliseconds() < 500);

            //Pivot to starting/calibration point
            while(drive.pivot(0, 305, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 500,
                    5, Direction.FASTEST) && opModeIsActive());

            drive.stop();

            while(!gamepad1.a && opModeIsActive()){
                telemetry.addData("Status", "Press Gamepad 1 A to Repeat");
                telemetry.update();
            }
        }
    }

    private void setMotorBehaviors(){
        //Hardware Map
        right_front = hardwareMap.dcMotor.get("rf");
        right_back = hardwareMap.dcMotor.get("rb");
        left_front = hardwareMap.dcMotor.get("lf");
        left_back = hardwareMap.dcMotor.get("lb");

        //Set motor behaviors
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        verticalLeft = hardwareMap.dcMotor.get("rf");
        verticalRight = hardwareMap.dcMotor.get("rb");
        horizontal = hardwareMap.dcMotor.get("lf");
        horizontal2 = hardwareMap.dcMotor.get("lb");

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

        //Status update
        telemetry.addData("Status", "Motor Hardware Initialized");
        telemetry.update();
    }
}
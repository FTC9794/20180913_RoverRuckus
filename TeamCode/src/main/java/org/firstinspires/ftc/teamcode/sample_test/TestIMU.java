package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

/**
 * Created by Sarthak on 9/11/2018.
 */
@Autonomous(name = "Test IMU Values", group = "Test")
public class TestIMU extends LinearOpMode{

    IIMU imu;
    BNO055IMU boschIMU;

    @Override
    public void runOpMode() throws InterruptedException {
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.initialize();
        imu.setOffset(0);
        telemetry.addData("Status", "IMU Instantiated");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Angle", imu.getZAngle());
            telemetry.update();
        }
    }
}

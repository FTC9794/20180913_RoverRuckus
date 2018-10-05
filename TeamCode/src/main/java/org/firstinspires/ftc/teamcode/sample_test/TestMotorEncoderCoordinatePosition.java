package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;

/**
 * Created by Sarthak on 10/1/2018.
 */
@Autonomous(name = "Motor Encoder Coordinate Position", group = "Test")
public class TestMotorEncoderCoordinatePosition extends LinearOpMode {

    DcMotor verticalRight, verticalLeft, horizontal;
    double vrPos = 0, vlPos = 0, hPos = 0;


    final double COUNTS_PER_INCH_Y = 307.699557;
    final double COUNTS_PER_INCH_X = 307.699557;


    //Orientation, Magnitude, X, Y
    double x = 0, y = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        verticalLeft = hardwareMap.dcMotor.get("vl");
        verticalRight = hardwareMap.dcMotor.get("vr");
        horizontal = hardwareMap.dcMotor.get("h");

        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()){
            vlPos = verticalLeft.getCurrentPosition();
            vrPos = verticalRight.getCurrentPosition();
            hPos = horizontal.getCurrentPosition();

            y = ((vlPos + vrPos) / 2) / COUNTS_PER_INCH_Y;
            x = hPos / COUNTS_PER_INCH_X;

            double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));


            telemetry.addData("Vertical Right Position", vrPos);
            telemetry.addData("Vertical Left Position", vlPos);
            telemetry.addData("Horizontal Position", hPos);
            telemetry.addData("x position", x);
            telemetry.addData("y position", y);
            telemetry.addData("Distance traveled", distance);

            telemetry.update();
        }
    }
}

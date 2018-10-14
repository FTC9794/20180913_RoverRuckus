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


    final double COUNTS_PER_INCH = 307.699557;
    final double UNITS_TO_DEGREES = 0.0064033333;


    //Orientation, Magnitude, X, Y
    double x = 0, y = 0;

    double prevRight = 0, prevLeft = 0;
    double length = 13.25 * COUNTS_PER_INCH;

    double changeInPosition = 0, changeInAngle = 0;
    double changeInX = 0, changeInY = 0;
    double position = 0, angle = 0;

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
            //Get Current Positions
            vlPos = verticalLeft.getCurrentPosition();
            vrPos = verticalRight.getCurrentPosition();

            double leftChange = vlPos - prevLeft;
            double rightChange = vrPos - prevRight;

            //Calculate Angle
            changeInAngle = (leftChange - rightChange) / (length);
            //Add change in angle to cumulative angle
            angle = ((angle + changeInAngle) % 360);

            //Calculate x and y position
            changeInPosition = (leftChange + rightChange) / 2;
            changeInX = changeInPosition * Math.sin(angle + (changeInAngle/2));
            changeInY = changeInPosition * Math.cos(angle + (changeInAngle/2));

            x += changeInX;
            y += changeInY;

            //Update vars
            prevLeft = vlPos;
            prevRight = vrPos;

            //Display calculations
            telemetry.addData("Vertical Right Position", vrPos);
            telemetry.addData("Vertical Left Position", vlPos);
            telemetry.addData("Angle Radians", angle);
            telemetry.addData("Angle to Degrees", Math.toDegrees(angle));
            telemetry.addData("X Position", x / COUNTS_PER_INCH);
            telemetry.addData("Y Position", y / COUNTS_PER_INCH);
            telemetry.update();
        }
    }

    void distanceFormula(){
        //Get Current Positions
        vlPos = verticalLeft.getCurrentPosition();
        vrPos = verticalRight.getCurrentPosition();
        hPos = horizontal.getCurrentPosition();

        //Average the Vertical Wheels
        y = ((vlPos + vrPos) / 2) / COUNTS_PER_INCH;
        //Convert the horizontal wheel counts into inches
        x = hPos / COUNTS_PER_INCH;

        //Calculate distance
        double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));

        //Display calculations
        telemetry.addData("Vertical Right Position", vrPos);
        telemetry.addData("Vertical Left Position", vlPos);
        telemetry.addData("Horizontal Position", hPos);
        telemetry.addData("x position", x);
        telemetry.addData("y position", y);
        telemetry.addData("Distance traveled", distance);

        telemetry.update();
    }
}

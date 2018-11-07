package org.firstinspires.ftc.teamcode.sample_test;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.R;

/**
 * Created by Sarthak on 10/1/2018.
 */
@Autonomous(name = "Motor Encoder Coordinate Position", group = "Test")
public class TestMotorEncoderCoordinatePosition extends LinearOpMode {

    DcMotor verticalRight, verticalLeft, horizontal;

    double vrPos = 0, vlPos = 0, hPos = 0;
    final double alpha = 53.13;

    SoundPool sound;
    int beepID;

    final double COUNTS_PER_INCH = 307.699557;
    final double UNITS_TO_DEGREES = 0.0064033333;


    //Orientation, Magnitude, X, Y
    double x = 0, y = 0;

    double prevRight = 0, prevLeft = 0, prevHorizontal = 0;
    double length = 13.25 * COUNTS_PER_INCH;

    double changeInPosition = 0, changeInAngle = 0;
    double changeInX = 0, changeInY = 0;
    double position = 0, angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        sound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
        beepID = sound.load(hardwareMap.appContext, R.raw.supermariobros, 1);

        verticalLeft = hardwareMap.dcMotor.get("right_front");
        verticalRight = hardwareMap.dcMotor.get("right_back");
        horizontal = hardwareMap.dcMotor.get("left_front");

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

        sound.play(beepID, 1, 1, 1, 0, 1);

        while(opModeIsActive()){
            algorithm2();
            telemetry.update();
        }
    }

    void algorithm2(){
        //Get Current Positions
        vlPos = verticalLeft.getCurrentPosition();
        vrPos = verticalRight.getCurrentPosition();
        hPos = horizontal.getCurrentPosition();

        double leftChange = vlPos - prevLeft;
        double rightChange = vrPos - prevRight;
        double horizontalChange = hPos - prevHorizontal;

        //Change in angle
        changeInAngle = (leftChange - rightChange) / (length);
        angle = ((angle + changeInAngle));

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange + (((leftChange-rightChange)/2) * Math.sin(alpha));
        x = x + (p*Math.sin(angle) + n*Math.cos(angle));
        y = y + (p*Math.cos(angle) - n*Math.sin(angle));

        prevLeft = vlPos;
        prevRight = vrPos;
        prevHorizontal = hPos;

        telemetry.addData("Vertical Right Position", vrPos);
        telemetry.addData("Vertical Left Position", vlPos);
        telemetry.addData("Horizontal Position", hPos);
        //telemetry.addData("Angle Radians", angle);
        telemetry.addData("Algorithm 2 Angle (Degrees)", Math.toDegrees(angle) % 360);
        telemetry.addData("Algorithm 2 X Position", x / COUNTS_PER_INCH);
        telemetry.addData("Algorithm 2 Y Position", y / COUNTS_PER_INCH);
    }

    void algorithm1(){
        //Get Current Positions
        vlPos = verticalLeft.getCurrentPosition();
        vrPos = verticalRight.getCurrentPosition();
        hPos = horizontal.getCurrentPosition();

        double leftChange = vlPos - prevLeft;
        double rightChange = vrPos - prevRight;
        double horizontalChange = hPos - prevHorizontal;

        //Calculate Angle
        changeInAngle = (leftChange - rightChange) / (length);
        //Add change in angle to cumulative angle
        angle = ((angle + changeInAngle));

        //Calculate x and y position
        changeInPosition = (leftChange + rightChange) / 2;
        changeInX = (changeInPosition * Math.sin(angle + (changeInAngle/2))) + (horizontalChange * Math.cos(angle + (changeInAngle/2)));
        changeInY = (changeInPosition * Math.cos(angle + (changeInAngle/2))) + (-horizontalChange * Math.sin(angle + (changeInAngle/2)));


        x += changeInX;
        y += changeInY;

        //Update vars
        prevLeft = vlPos;
        prevRight = vrPos;
        prevHorizontal = hPos;

        //Display calculations
        telemetry.addData("Vertical Right Position", vrPos);
        telemetry.addData("Vertical Left Position", vlPos);
        telemetry.addData("Horizontal Position", hPos);
        //telemetry.addData("Angle Radians", angle);
        telemetry.addData("Algorithm 1 Angle to Degrees", Math.toDegrees(angle) % 360);
        telemetry.addData("Algorithm 1 X Position", x / COUNTS_PER_INCH);
        telemetry.addData("Algorithm 1 Y Position", y / COUNTS_PER_INCH);
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

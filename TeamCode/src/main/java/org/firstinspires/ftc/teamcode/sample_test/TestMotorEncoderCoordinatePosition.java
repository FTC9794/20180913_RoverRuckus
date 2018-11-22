package org.firstinspires.ftc.teamcode.sample_test;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.DataLogger;
import org.firstinspires.ftc.teamcode.R;

import java.util.Date;

/**
 * Created by Sarthak on 10/1/2018.
 */
@TeleOp(name = "Motor Encoder Coordinate Position", group = "Test")
public class TestMotorEncoderCoordinatePosition extends LinearOpMode {

    DcMotor verticalRight, verticalLeft, horizontal;

    double vrPos = 0, vlPos = 0, hPos = 0;
    final double alpha = 20.63;

    SoundPool sound;
    int beepID;

    final double COUNTS_PER_INCH = 307.699557;
    final double UNITS_TO_DEGREES = 0.0064033333;


    //Orientation, Magnitude, X, Y
    double x = 0*COUNTS_PER_INCH, y = 0*COUNTS_PER_INCH;

    double prevRight = 0, prevLeft = 0, prevHorizontal = 0;
    double length = 12.75 * COUNTS_PER_INCH;

    double changeInPosition = 0, changeInAngle = 0;
    double changeInX = 0, changeInY = 0;
    double position = 0, angle = 0;

    DataLogger data;
    Date date;

    @Override
    public void runOpMode() throws InterruptedException {
        sound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
        beepID = sound.load(hardwareMap.appContext, R.raw.supermariobros, 1);

        verticalLeft = hardwareMap.dcMotor.get("rf");
        verticalRight = hardwareMap.dcMotor.get("rb");
        horizontal = hardwareMap.dcMotor.get("lf");

        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        date = new Date();
        data = new DataLogger(date.toString() + "Encoder Position Calculations");
        data.addField("Left Change");
        data.addField("Right Change");
        data.addField("Horizontal Change");
        data.addField("Change in Angle");
        data.addField("Angle");
        data.addField("p");
        data.addField("n");
        data.addField("x");
        data.addField("y");
        data.newLine();



        waitForStart();

        //sound.play(beepID, 1, 1, 1, 0, 1);

        while(opModeIsActive()){
            globalCoordinatePositionUpdate();
            telemetry.update();
        }

        data.closeDataLogger();
    }

    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        vlPos = verticalLeft.getCurrentPosition();
        vrPos = verticalRight.getCurrentPosition();
        hPos = horizontal.getCurrentPosition();

        double leftChange = vlPos - prevLeft;
        double rightChange = vrPos - prevRight;
        double horizontalChange = hPos - prevHorizontal;

        //Calculate Angle
        changeInAngle = (leftChange - rightChange) / (length);
        angle = ((angle + changeInAngle));

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange + (((leftChange-rightChange)/2) * Math.sin(alpha));
        x = x + (p*Math.sin(angle) + n*Math.cos(angle));
        y = y + -(p*Math.cos(angle) - n*Math.sin(angle));

        prevLeft = vlPos;
        prevRight = vrPos;
        prevHorizontal = hPos;

        data.addField((float) leftChange);
        data.addField((float) rightChange);
        data.addField((float) horizontalChange);
        data.addField((float) changeInAngle);
        data.addField((float) angle);
        data.addField((float) p);
        data.addField((float) n);
        data.addField((float) x);
        data.addField((float) y);
        data.newLine();

        telemetry.addData("Vertical Right Position", vrPos);
        telemetry.addData("Vertical Left Position", vlPos);
        telemetry.addData("Horizontal Position", hPos);
        telemetry.addData("Angle (Degrees)", Math.toDegrees(angle) % 360);
        telemetry.addData("X Position", x / COUNTS_PER_INCH);
        telemetry.addData("Y Position", y / COUNTS_PER_INCH);
    }
}

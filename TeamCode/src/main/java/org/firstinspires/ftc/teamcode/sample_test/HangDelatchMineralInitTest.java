package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.team_marker.ITeamMarker;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.ServoArmDrop;

import java.util.ArrayList;

/**
 * Created by Sarthak on 1/5/2019.
 */
@Autonomous (name = "Hang Delatch and Mineral Mechanism Positioning Test", group = "Test")
public class HangDelatchMineralInitTest extends LinearOpMode {
    DcMotor right_front, right_back, left_front, left_back;
    DcMotor mineral_rotation;
    DcMotor intakeRotation;
    DcMotor verticalLeft, verticalRight, horizontal, horizontal2;
    ArrayList motors, encoders;

    DcMotor hang;

    ModernRoboticsI2cRangeSensor leftWallPing;

    DigitalChannel rotation_limit;

    Servo hang_latch;

    ITeamMarker teamMarker;
    Servo teamMarkerServo;

    Servo scanner;

    //Declare OpMode timers
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //Init motor hardware map and behaviors
        setMotorBehaviors();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();

        runtime.reset();

        /**
         * *****************************************************************************************
         * *****************************************************************************************
         * *******************************OPMODE RUNS HERE******************************************
         * *****************************************************************************************
         * *****************************************************************************************
         */
        //Release Hang Latch
        hang_latch.setPosition(1);
        waitMilliseconds(750, runtime);

        //Delatch from hanger
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineral_rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setTargetPosition(4900);
        hang.setPower(1);
        mineral_rotation.setTargetPosition(170);
        mineral_rotation.setPower(1);
        intakeRotation.setTargetPosition(250);
        intakeRotation.setPower(1);

        runtime.reset();
        while(hang.isBusy() && opModeIsActive()){
            if(!mineral_rotation.isBusy()){
                if(rotation_limit.getState()){
                    mineral_rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    mineral_rotation.setPower(-0.15);
                }else{
                    mineral_rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    mineral_rotation.setPower(0);
                }
            }
            telemetry.addData("hang pos", hang.getCurrentPosition());
            telemetry.update();
        }
        hang.setPower(0);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive()){
            telemetry.addData("Hang Current Position", hang.getCurrentPosition());
            telemetry.addData("Mineral Rotation Current Position", mineral_rotation.getCurrentPosition());
            telemetry.addData("Intake Rotation Current Position", intakeRotation.getCurrentPosition());
            telemetry.update();
        }
    }

    /**
     * Stop all actions for a specified amount of time (in milliseconds)
     * @param milliseconds amount of time to wait
     * @param timer ElapsedTimer object to keep track of the time
     */
    private void waitMilliseconds(double milliseconds, ElapsedTime timer){
        //Reset the timer
        timer.reset();
        //Wait until the time inputted has fully elapsed
        while(opModeIsActive() && timer.milliseconds() < milliseconds);
    }

    /**
     * Setup the hardware map and the motor modes, zero power behaviors, and direction
     */
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

        mineral_rotation = hardwareMap.dcMotor.get("mineral_rotation");
        mineral_rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineral_rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineral_rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeRotation = hardwareMap.dcMotor.get("intake_rotation");
        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        teamMarkerServo = hardwareMap.servo.get("marker_servo");
        teamMarker = new ServoArmDrop(teamMarkerServo);
        teamMarker.hold();

        scanner = hardwareMap.servo.get("scanner");
        scanner.setPosition(0.5);

        rotation_limit = hardwareMap.digitalChannel.get("rotation_limit");

        hang_latch = hardwareMap.servo.get("hang_stopper");
        hang_latch.setPosition(0);

        leftWallPing = (ModernRoboticsI2cRangeSensor) hardwareMap.get("left_us");

        hang = hardwareMap.dcMotor.get("hang");
        hang.setDirection(DcMotorSimple.Direction.REVERSE);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
}

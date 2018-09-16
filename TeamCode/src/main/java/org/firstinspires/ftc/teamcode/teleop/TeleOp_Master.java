package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Sarthak on 9/7/2018.
 */
@TeleOp(name = "Master Teleop", group = "Teleop")
@Disabled
public class TeleOp_Master extends LinearOpMode {

    DcMotor right;
    DcMotor left;

    DcMotor latchMotor;

    Servo delatch;

    Servo intakeRotation;
    DcMotor intakeArm;

    int intakeArmEncoderPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        initHardwareMap();
        setMotorBehaviors();
        setServoPositions();

        waitForStart();

        //Drivetrain controls
        double leftPower = -gamepad1.left_stick_y + gamepad1.right_stick_x;
        double rightPower = -gamepad1.left_stick_y - gamepad1.right_stick_x;
        left.setPower(leftPower);
        right.setPower(rightPower);

        //Intake arm controls
        if(gamepad2.y){
            intakeArmEncoderPosition += 5;
        }else if(gamepad2.a){
            intakeArmEncoderPosition -= 5;
        }
        if(intakeArmEncoderPosition < 0){
            intakeArmEncoderPosition = 0;
        }
        intakeArm.setTargetPosition(intakeArmEncoderPosition);
        intakeArm.setPower(1);

        if(gamepad2.dpad_left){
            intakeRotation.setPosition(0);
        }else if(gamepad2.dpad_right){
            intakeRotation.setPosition(1);
        }else if(gamepad2.dpad_up){
            if(intakeRotation.getPosition() + 0.05 < 1) {
                intakeRotation.setPosition(intakeRotation.getPosition() + 0.05);
            }else{
                intakeRotation.setPosition(1);
            }
        }else if(gamepad2.dpad_down){
            if(intakeRotation.getPosition() - 0.05 < 0) {
                intakeRotation.setPosition(intakeRotation.getPosition() - 0.05);
            }else{
                intakeRotation.setPosition(0);
            }
        }


        //Telemetry Data
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Latch Encoder Position", latchMotor.getCurrentPosition());
    }

    private void initHardwareMap(){
        //Hardware Map
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");
        delatch = hardwareMap.servo.get("delatch");
        intakeArm = hardwareMap.dcMotor.get("intake_arm");
        intakeRotation = hardwareMap.servo.get("intake_rotation");
        telemetry.addData("Status", "Hardware Map Completed");
        telemetry.update();
    }

    private void setMotorBehaviors(){
        //Set motor behaviors
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Set Zero Power Behavior
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Reverse Right Motor
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Motor Hardware Initialized");
        telemetry.update();
    }

    private void setServoPositions(){
        intakeRotation.setPosition(0);
        telemetry.addData("Status", "Servo Positions Set");
    }


}

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.latching.ILatching;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.ITeamMarker;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.ServoArmDrop;

import java.util.ArrayList;

/**
 * Created by Sarthak on 9/7/2018.
 */
@TeleOp(name = "Master Teleop", group = "Teleop")
public class TeleOp_Master extends LinearOpMode {

    DcMotor right;
    DcMotor left;
    IDrivetrain drive;

    Servo teamMarkerServo;
    ITeamMarker teamMarker;

    DcMotor latchMotor;
    ILatching latch;

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

        //Telemetry Data
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Latch Encoder Position", latchMotor.getCurrentPosition());
    }

    private void initHardwareMap(){
        //Hardware Map
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");
        teamMarkerServo = hardwareMap.servo.get("marker");
        telemetry.addData("Status", "Hardware Map Completed");
        telemetry.update();
    }

    private void setMotorBehaviors(){
        //Set motor behaviors
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        teamMarker = new ServoArmDrop(teamMarkerServo);
        teamMarker.hold();
        telemetry.addData("Status", "Servo Positions Set");
    }


}

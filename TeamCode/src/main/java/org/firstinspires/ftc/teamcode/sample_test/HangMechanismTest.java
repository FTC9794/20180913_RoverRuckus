package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sarthak on 1/26/2019.
 */
@TeleOp(name = "Hang Mechanism (Hang and Delatch) Test", group = "Test")
public class HangMechanismTest extends LinearOpMode {
    DcMotor hang;
    final int latchPosition = 3560, hangPosition = 1750, startPosition = 50;
    int numTests = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        hang = hardwareMap.dcMotor.get("hang");

        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hang.setDirection(DcMotorSimple.Direction.REVERSE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean selected = false;
        while(!selected && !isStopRequested()){
            if(gamepad1.a){
                numTests++;
                while(gamepad1.a && !isStopRequested());
            }else if(gamepad1.b){
                numTests--;
                while(gamepad1.b && !isStopRequested());
            }else if(gamepad1.x){
                selected = true;
            }
            telemetry.addData("Number of Tests", numTests);
            telemetry.addData("Gamepad 1 A", "Increase Number of Extends");
            telemetry.addData("Gamepad 1 B", "Decrease Number of Extends");
            telemetry.addData("Exit", "Gamepad 1 X");
            telemetry.update();
        }

        telemetry.addData("Diagnostic Test", "Hang");
        telemetry.addData("Number of Extensions", numTests);
        telemetry.update();

        waitForStart();

        int i = 0;
        while(i < numTests && opModeIsActive()){
            hang.setTargetPosition(latchPosition);
            hang.setPower(1);
            while(hang.isBusy() && opModeIsActive()){
                telemetry.addData("Hang Diagnostic", "Moving to Latch Position");
                telemetry.addData("Hang Encoder Position", hang.getCurrentPosition());
                telemetry.update();
            }

            while(gamepad1.atRest() && opModeIsActive()){
                telemetry.addData("Hang Diagnostic", "Move Joysticks to Proceed to Hang Position");
                telemetry.update();
            }

            hang.setTargetPosition(hangPosition);
            hang.setPower(1);

            while(hang.isBusy() && opModeIsActive()){
                telemetry.addData("Hang Diagnostic", "Moving to Hang Position");
                telemetry.addData("Hang Encoder Position", hang.getCurrentPosition());
                telemetry.update();
            }

            while(gamepad1.atRest() && opModeIsActive()){
                telemetry.addData("Hang Diagnostic", "Move Joysticks to Reset Position");
                telemetry.update();
            }

            hang.setTargetPosition(startPosition);
            hang.setPower(1);

            while(hang.isBusy() && opModeIsActive()){
                telemetry.addData("Hang Diagnostic", "Moving to Start Position");
                telemetry.addData("Hang Encoder Position", hang.getCurrentPosition());
                telemetry.update();
            }

            while(gamepad1.atRest() && opModeIsActive()){
                telemetry.addData("Hang Diagnostic", "Move Joysticks to Proceed to Latch Position");
                telemetry.update();
            }

            i++;
        }
    }
}

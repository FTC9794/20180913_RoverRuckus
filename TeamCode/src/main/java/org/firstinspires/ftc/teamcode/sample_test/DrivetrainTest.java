package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sarthak on 1/30/2019.
 */
@TeleOp(name = "Drivetrain Test", group = "Test")
public class DrivetrainTest extends LinearOpMode {
    DcMotor rf, rb, lf, lb;
    ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime = new ElapsedTime();

        telemetry.addData("Status", "Init Complete");
        telemetry.addLine("Drivetrain Test");

        waitForStart();
        runtime.reset();

        setPowerAll(1, 1, 1, 1);
        waitMilliseconds(3000, runtime);

        setPowerAll(-1, -1, -1, -1);
        waitMilliseconds(3000, runtime);

        setPowerAll(1, 1, -1, -1);
        waitMilliseconds(3000, runtime);

        setPowerAll(-1, -1, 1, 1);
        waitMilliseconds(3000, runtime);

        setPowerAll(0, 0, 0, 0);

        while(opModeIsActive()){
            if(gamepad1.a){
                setPowerAll(1, 0, 0, 0);
            }else if(gamepad1.dpad_down){
                setPowerAll(-1, 0, 0, 0);
            }else if(gamepad1.b){
                setPowerAll(0, 1, 0, 0);
            }else if(gamepad1.dpad_right){
                setPowerAll(0, -1, 0, 0);
            }else if(gamepad1.x){
                setPowerAll(0, 0, 1, 0);
            }else if(gamepad1.dpad_left){
                setPowerAll(0, 0, -1, 0);
            }else if(gamepad1.y){
                setPowerAll(0, 0, 0, 1);
            }else if(gamepad1.dpad_up){
                setPowerAll(0, 0, 0, -1);
            }else{
                setPowerAll(0, 0, 0, 0);
            }
            telemetry.addLine("Press Gamepad 1 A -- RF Power 1");
            telemetry.addLine("Press Gamepad 1 B -- RB Power 1");
            telemetry.addLine("Press Gamepad 1 X -- LF Power 1");
            telemetry.addLine("Press Gamepad 1 Y -- LB Power 1");
            telemetry.addLine("Press Gamepad 1 UP -- RF Power -1");
            telemetry.addLine("Press Gamepad 1 DOWN -- RB Power -1");
            telemetry.addLine("Press Gamepad 1 LEFT -- LF Power -1");
            telemetry.addLine("Press Gamepad 1 RIGHT -- LB Power -1");
            telemetry.update();
        }
    }

    private void setPowerAll(double rfPower, double rbPower, double lfPower, double lbPower){
        rf.setPower(rfPower);
        rb.setPower(rbPower);
        lf.setPower(lfPower);
        lb.setPower(lbPower);
    }

    private void waitMilliseconds(double milliseconds, ElapsedTime timer){
        //Reset the timer
        timer.reset();
        //Wait until the time inputted has fully elapsed
        while(opModeIsActive() && timer.milliseconds() < milliseconds);
    }
}

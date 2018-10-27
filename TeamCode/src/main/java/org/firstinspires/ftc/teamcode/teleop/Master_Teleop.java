package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sarthak on 10/26/2018.
 */
@TeleOp(name = "Master Teleop", group = "Teleop")
public class Master_Teleop extends LinearOpMode {
    DcMotor rf, rb, lf, lb;

    @Override
    public void runOpMode() throws InterruptedException {
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            double pitch = -gamepad1.left_stick_y;
            double roll = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;

            rf.setPower(pitch-roll+pivot);
            rb.setPower(pitch+roll+pivot);
            lf.setPower(pitch+roll-pivot);
            lb.setPower(pitch-roll-pivot);

        }
    }
}

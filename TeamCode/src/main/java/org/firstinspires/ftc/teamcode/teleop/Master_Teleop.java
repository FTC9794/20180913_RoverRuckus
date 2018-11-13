package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;

/**
 * Created by Sarthak on 10/26/2018.
 */
@TeleOp(name = "Master Teleop", group = "Teleop")
public class Master_Teleop extends LinearOpMode {
    DcMotor rf, rb, lf, lb;

    DcMotor hang;
    int hangPosition = 0;

    GamepadWrapper Gamepad1 = new GamepadWrapper(gamepad1);
    GamepadWrapper Gamepad2 = new GamepadWrapper(gamepad2);

    @Override
    public void runOpMode() throws InterruptedException {
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        hang = hardwareMap.dcMotor.get("hang");
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()){
            //Drivetrain controls
            double pitch = Gamepad1.leftStickY();
            double roll = Gamepad1.leftStickX();
            double pivot = Gamepad1.rightStickX();;

            rf.setPower(pitch-roll+pivot);
            rb.setPower(pitch+roll+pivot);
            lf.setPower(pitch+roll-pivot);
            lb.setPower(pitch-roll-pivot);

            //Hanging mechanism controls
            if(Gamepad2.rightTrigger() > 0.2){
                hangPosition += 10;
            }else if(Gamepad2.leftTrigger() > 0.2){
                hangPosition -= 10;
            }
            if(hangPosition < 0){
                hangPosition = 0;
            }
            hang.setTargetPosition(hangPosition);
        }
    }
}

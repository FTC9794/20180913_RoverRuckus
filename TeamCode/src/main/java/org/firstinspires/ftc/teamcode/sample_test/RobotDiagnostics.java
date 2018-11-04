package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sarthak on 10/28/2018.
 */
@TeleOp(name = "Robot Diagnostics", group = "Diagnostics")
public class RobotDiagnostics extends LinearOpMode {

    DcMotor rf, rb, lf, lb;

    int diagnostic = 0;
    final int maxDiagnosticSelection = 0;

    final String[] diagnosticTests = {
        "Drivetrain Motors Test", "Lift Mechanism Test"
    };

    @Override
    public void runOpMode() throws InterruptedException {

        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean selectedTest = false;
        while(!selectedTest && !isStopRequested()){
            if(gamepad1.dpad_up){
                diagnostic++;
                while(gamepad1.dpad_up && isStarted());
            }else if (gamepad1.dpad_down){
                diagnostic --;
                while(gamepad1.dpad_down && isStarted());
            }
            if(diagnostic > maxDiagnosticSelection){
                diagnostic = maxDiagnosticSelection;
            }else if(diagnostic < 0){
                diagnostic = maxDiagnosticSelection;
            }

            if(gamepad1.a){
                selectedTest = true;
            }

            telemetry.addData("Diagnostic Test", diagnosticTests[diagnostic]);
            telemetry.addData("Confirm Selection", "Press A");
            telemetry.addData("Selected Test", selectedTest);
            telemetry.update();
        }

        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            switch (diagnostic){
                case 0: //Drive motor diagnostic
                    double power = -gamepad1.right_stick_y;
                    rf.setPower(power);
                    rb.setPower(power);
                    lf.setPower(power);
                    lb.setPower(power);
                    telemetry.addData("Power", power);
                    break;
            }
        }

    }
}

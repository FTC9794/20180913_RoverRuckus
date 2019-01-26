package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sarthak on 1/25/2019.
 */
@TeleOp(name = "Mineral Extension Test", group = "Test")
public class MineralExtensionTest extends LinearOpMode {
    DcMotor mineralExtension;

    int numberOfExtends = 25;
    final int extendPosition = 2600, inPosition = 50;

    ElapsedTime runTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        mineralExtension = hardwareMap.dcMotor.get("mineral_extension");
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineralExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean selected = false;
        while(!selected && !isStopRequested()){
            if(gamepad1.a){
                numberOfExtends++;
                while(gamepad1.a && !isStopRequested());
            }else if(gamepad1.b){
                numberOfExtends--;
                while(gamepad1.b && !isStopRequested());
            }else if(gamepad1.x){
                selected = true;
            }
            telemetry.addData("Number of Extends", numberOfExtends);
            telemetry.addData("Gamepad 1 A", "Increase Number of Extends");
            telemetry.addData("Gamepad 1 B", "Decrease Number of Extends");
            telemetry.addData("Exit", "Gamepad 1 X");
            telemetry.update();
        }

        telemetry.addData("Diagnostic Test", "Mineral Extension");
        telemetry.addData("Number of Extensions", numberOfExtends);
        telemetry.update();

        waitForStart();

        int numExtends = 0;
        while(numExtends < numberOfExtends && opModeIsActive()){
            mineralExtension.setTargetPosition(extendPosition);
            mineralExtension.setPower(1);
            while(mineralExtension.isBusy() && opModeIsActive());

            runTime.reset();
            while(runTime.milliseconds() < 250 && opModeIsActive());

            mineralExtension.setTargetPosition(inPosition);
            mineralExtension.setPower(1);
            while(mineralExtension.isBusy() && opModeIsActive());

            runTime.reset();
            while(runTime.milliseconds() < 250 && opModeIsActive());

            numExtends++;

            telemetry.addData("Number of Extends", String.valueOf(numExtends) + "/" + String.valueOf(numberOfExtends));
            telemetry.update();
        }

        mineralExtension.setPower(0);
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive()){
            telemetry.addData("Diagnostic Test", "Mineral Extension Test Complete");
            telemetry.addData("Number of Extends", numberOfExtends);
            telemetry.update();
        }
    }
}

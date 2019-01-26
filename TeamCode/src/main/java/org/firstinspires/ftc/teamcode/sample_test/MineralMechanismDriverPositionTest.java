package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sarthak on 1/26/2019.
 */
public class MineralMechanismDriverPositionTest extends LinearOpMode {
    DcMotor mineralExtension, mineralRotation;

    int extensionMaxPosition = 2700, extensionDumpPositionBalls = 1400,
            extensionDumpPositionBlocks = 2000,
            rotationExtendPosition = 650, mineralRotationDumpBallPosition = 1000, mineralRotationDumpBlocksPosition = 1100, mineralRotationIncrement = 6,
            rotationMaxPosition = 1200, rotationDrivePosition = 390;

    final double mineralRotationPower = 0.2, mineralExtensionPower = 1;

    int numTests = 5;
    @Override
    public void runOpMode() throws InterruptedException {
        mineralExtension = hardwareMap.dcMotor.get("mineral_extension");
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineralExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mineralRotation = hardwareMap.dcMotor.get("mineral_rotation");
        mineralRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        telemetry.addData("Diagnostic Test", "Mineral Mechanism Driver Position & Enhancements Test");
        telemetry.addData("Number of Tests", numTests);
        telemetry.update();

        waitForStart();

        //Drive Position
        mineralExtension.setTargetPosition(0);
        mineralRotation.setTargetPosition(rotationDrivePosition);
        mineralExtension.setPower(mineralExtensionPower);
        mineralRotation.setPower(mineralRotationPower);
        while(mineralExtension.isBusy() && mineralRotation.isBusy() && opModeIsActive()){
            telemetry.addData("Mineral Mechanism Driver Position Test", "Moving to Drive Position");
            telemetry.update();
        }

        while(gamepad1.atRest() && opModeIsActive()){
            telemetry.addData("Mineral Mechanism Driver Position Test", "Move Joysticks to Proceed to Intake Position");
            telemetry.update();
        }

        int i = 0;
        while(i < numTests && opModeIsActive()){
            //Intake Position
            mineralRotation.setTargetPosition(0);
            mineralRotation.setPower(mineralRotationPower);
            while(mineralRotation.isBusy() && opModeIsActive()){
                telemetry.addData("Mineral Mechanism Driver Position Test", "Moving to Intake Position");
                telemetry.update();
            }

            while(gamepad1.atRest() && opModeIsActive()){
                telemetry.addData("Mineral Mechanism Driver Position Test", "Move Joysticks to Proceed to Deposit Position");
                telemetry.update();
            }

            //Deposit Position
            mineralExtension.setTargetPosition(extensionDumpPositionBalls);
            mineralExtension.setPower(mineralExtensionPower);
            while(mineralExtension.getCurrentPosition() < (extensionDumpPositionBalls/4) && opModeIsActive()){
                telemetry.addData("Mineral Mechanism Driver Position Test", "Moving to Deposit Position");
                telemetry.update();
            }
            mineralRotation.setTargetPosition(mineralRotationDumpBallPosition);
            mineralRotation.setPower(mineralRotationPower);
            while(mineralExtension.isBusy() && mineralRotation.isBusy() && opModeIsActive()){
                telemetry.addData("Mineral Mechanism Driver Position Test", "Moving to Deposit Position");
                telemetry.update();
            }
            mineralRotation.setPower(0);

            while(gamepad1.atRest() && opModeIsActive()){
                telemetry.addData("Mineral Mechanism Driver Position Test", "Move Joysticks to Proceed to Drive Position");
                telemetry.update();
            }

            mineralRotation.setTargetPosition(rotationDrivePosition);
            mineralRotation.setPower(mineralRotationPower);
            while(mineralRotation.getCurrentPosition() > rotationExtendPosition && opModeIsActive()){
                telemetry.addData("Mineral Mechanism Driver Position Test", "Moving to Drive Position");
                telemetry.update();
            }
            mineralExtension.setTargetPosition(0);
            mineralExtension.setPower(mineralExtensionPower);
            while(mineralRotation.isBusy() && mineralExtension.isBusy() && opModeIsActive()){
                telemetry.addData("Mineral Mechanism Driver Position Test", "Moving to Drive Position");
                telemetry.update();
            }

            i++;
        }
    }
}

package org.firstinspires.ftc.teamcode.sample_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.sample_test.MineralMechanismDriverPositionTest.depositingPositionState.ROTATION1;
import static org.firstinspires.ftc.teamcode.sample_test.MineralMechanismDriverPositionTest.intakingPositionState.NOTHING;
import static org.firstinspires.ftc.teamcode.sample_test.MineralMechanismDriverPositionTest.intakingPositionState.ROTATING;

/**
 * Created by Sarthak on 1/26/2019.
 */
@TeleOp(name = "Mineral Mechanism Driver Position Test", group = "Test")
public class MineralMechanismDriverPositionTest extends LinearOpMode {

    Servo intakeGate;
    final double GATE_OPEN = 1, GATE_CLOSED = 0.425;

    /*

    Hang Variables

     */

    public enum intakingPositionState{NOTHING, EXTENDING, ROTATING};
    intakingPositionState intakePositionState = intakingPositionState.NOTHING;
    public enum depositingPositionState{NOTHING, INIT, ROTATION1, EXTENSIONINTAKEROTATION, ROTATION2};
    public enum depositingBlocksPositionState{NOTHING, INIT, ROTATION1, EXTENSIONINTAKEROTATION, ROTATION2};
    public enum drivingPositionState{NOTHING, ROTATION1, FINALPOSITION};
    depositingPositionState depositPositionState = depositingPositionState.NOTHING;
    depositingBlocksPositionState depositBlocksState = depositingBlocksPositionState.NOTHING;
    drivingPositionState drivePositionState = drivingPositionState.NOTHING;

    /*

    Mineral Mechanism rotation and extension variables

     */
    DcMotor mineralRotation, mineralExtension;
    DigitalChannel rotationLimit;

    /*

    Intake Mechanism variables

     */
    DcMotor intakeRotation;
    CRServo intake;
    int intakeDumpReadyPosition = 425, intakeDumpReadyPositionBlocks = 340, intakeIntakePosition = 535;
    final double intakeInPower = .73, intakeOutPower = -.73;
    double intakeRotationPower = .5;
    int intakeCurrentPosition;


    boolean intakePressed = false;
    boolean intaking = false;

    @Override
    public void runOpMode() throws InterruptedException {

        /*

        Mineral rotation and extension initialization

         */
        mineralRotation = hardwareMap.dcMotor.get("mineral_rotation");
        mineralExtension = hardwareMap.dcMotor.get("mineral_extension");
        rotationLimit = hardwareMap.digitalChannel.get("rotation_limit");

        mineralRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int mineralExtensionPosition, mineralRotationPosition;
        double mineralRotationPower;

        mineralExtensionPosition = 0;
        mineralRotationPosition = 0;

        int extensionMaxPosition = 2700, extensionDumpPositionBalls = 1580,
                extensionDumpPositionBlocks = 2190,
                rotationExtendPosition = 620, mineralRotationDumpBallPosition = 950, mineralRotationDumpBlocksPosition = 2140, mineralRotationIncrement = 6,
                rotationMaxPosition = 1200, rotationDrivePosition = 950;

        final double mineralExtensionPower = 1;
        /*

        Mineral Intake initialization

         */
        intake = hardwareMap.crservo.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRotation = hardwareMap.dcMotor.get("intake_rotation");


        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeCurrentPosition = 0;

        intakeGate = hardwareMap.servo.get("intake_gate");
        intakeGate.setPosition(GATE_OPEN);

        telemetry.addData("initialization", "done");
        telemetry.addData("intake rotation position", intakeRotation.getCurrentPosition());
        telemetry.update();

        mineralExtensionPosition = mineralExtension.getCurrentPosition();
        intakeCurrentPosition = intakeRotation.getCurrentPosition();

        telemetry.addLine("Init Complete");
        telemetry.addLine("Press Gamepad 1 B to go to Drive Position");
        telemetry.addLine("Press Gamepad 1 X to go to Intake Position");
        telemetry.addLine("Press Gamepad 1 A to go to Ball Deposit Position");
        telemetry.addLine("Press Gamepad 1 Y to go to Block Deposit Position");
        telemetry.addLine("Press Gamepad 1 Up to go to Open Gate");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            /*

            Mineral Rotation and extension code

             */

            if(gamepad2.right_bumper){
                mineralExtensionPosition = 0;
                intakeCurrentPosition = 0;
                mineralRotationPosition = 0;
                drivePositionState = drivingPositionState.NOTHING;
            }

            if(gamepad2.dpad_up){
                mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mineralExtensionPosition = mineralExtension.getCurrentPosition();

                if(mineralExtensionPosition>extensionMaxPosition){
                    mineralExtension.setPower(0);
                }else{
                    mineralExtension.setPower(mineralExtensionPower);
                }
                depositBlocksState = depositingBlocksPositionState.NOTHING;
                depositPositionState = depositingPositionState.NOTHING;
                intakePositionState = NOTHING;
                drivePositionState = drivingPositionState.NOTHING;

            }else if(gamepad2.dpad_down){
                mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mineralExtensionPosition = mineralExtension.getCurrentPosition();
                if(mineralExtensionPosition>0){
                    mineralExtension.setPower(-mineralExtensionPower);
                }else{
                    mineralExtension.setPower(0);
                }
                depositBlocksState = depositingBlocksPositionState.NOTHING;
                depositPositionState = depositingPositionState.NOTHING;
                intakePositionState = NOTHING;
                drivePositionState = drivingPositionState.NOTHING;

            }else{
                if(mineralExtension.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
                    mineralExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
                mineralExtension.setTargetPosition(mineralExtensionPosition);
                mineralExtension.setPower(1);

            }

            telemetry.addData("Extension Position", mineralExtension.getCurrentPosition());
            telemetry.addData("Rotation Position", mineralRotation.getCurrentPosition());

            mineralRotationPower = -gamepad2.right_stick_y;
            if(mineralRotationPower!=0){
                depositBlocksState = depositingBlocksPositionState.NOTHING;
                depositPositionState = depositingPositionState.NOTHING;
                intakePositionState = NOTHING;
                drivePositionState = drivingPositionState.NOTHING;
                if(mineralRotationPower<0&&!rotationLimit.getState()){
                    mineralRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mineralRotation.setPower(0);
                    mineralRotationPosition = 0;

                }else{
                    mineralRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(mineralRotationPower<0){

                        mineralRotationPosition = (int) (mineralRotationPosition-mineralRotationIncrement);
                    }else{
                        if(!(mineralRotationPosition>rotationMaxPosition)){
                            mineralRotationPosition = mineralRotationPosition+mineralRotationIncrement;
                        }

                    }
                    if(mineralRotation.getCurrentPosition() > mineralRotationPosition){
                        mineralRotation.setPower(0.175);
                    }else{
                        mineralRotation.setPower(0.175);
                    }
                    mineralRotation.setTargetPosition(mineralRotationPosition);

                }

            }else{
                if(!mineralRotation.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
                    mineralRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if(mineralRotation.getCurrentPosition() > mineralRotationPosition){
                    mineralRotation.setPower(0.175);
                }else{
                    mineralRotation.setPower(0.175);
                }

                mineralRotation.setTargetPosition(mineralRotationPosition);
            }

            /*

            Mineral Intake Code

             */

            if(intakeRotation.getCurrentPosition() < 100){
                intakeGate.setPosition(GATE_OPEN);
            }else if(gamepad1.dpad_up){
                intakeGate.setPosition(GATE_OPEN);
            }else{
                intakeGate.setPosition(GATE_CLOSED);
            }

            if(gamepad2.left_bumper){
                intakeIntakePosition = intakeRotation.getCurrentPosition();
                intakeDumpReadyPosition = intakeIntakePosition - 125;
            }

            if(gamepad1.left_trigger>.01){
                intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeCurrentPosition = intakeRotation.getCurrentPosition();
                intakeRotation.setPower(intakeRotationPower);
                depositBlocksState = depositingBlocksPositionState.NOTHING;
                depositPositionState = depositingPositionState.NOTHING;
                intakePositionState = NOTHING;
                drivePositionState = drivingPositionState.NOTHING;
            }else if(gamepad1.right_trigger>.01){
                intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeCurrentPosition = intakeRotation.getCurrentPosition();
                intakeRotation.setPower(-intakeRotationPower);
                depositBlocksState = depositingBlocksPositionState.NOTHING;
                depositPositionState = depositingPositionState.NOTHING;
                intakePositionState = NOTHING;
                drivePositionState = drivingPositionState.NOTHING;
            }else{
                if(intakeRotation.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
                    intakeRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
                intakeRotation.setTargetPosition(intakeCurrentPosition);
                intakeRotation.setPower(1);
            }



            if(gamepad1.right_bumper&&!intakePressed){
                intaking = !intaking;
                intakePressed = true;
            }else if(gamepad1.right_bumper){
                intakePressed = true;
            }else{
                intakePressed = false;
            }

            if(gamepad1.left_bumper){
                intake.setPower(intakeOutPower);
                intaking = false;
            }
            else if(intaking){
                intake.setPower(intakeInPower);
            }else if(!intaking){
                intake.setPower(0);
            }

            telemetry.addData("Intake Rotation Position", intakeRotation.getCurrentPosition());
            telemetry.update();



            /*

            State Machines

             */

            switch(intakePositionState){
                case NOTHING:
                    if(gamepad1.x){
                        intakePositionState = ROTATING;
                        intakeCurrentPosition = intakeIntakePosition;
                        mineralExtension.setTargetPosition(0);
                        mineralRotationPosition = 0;
                        depositBlocksState = depositingBlocksPositionState.NOTHING;
                        depositPositionState = depositingPositionState.NOTHING;
                        drivePositionState = drivingPositionState.NOTHING;
                        intaking = true;
                    }
                    break;

                case ROTATING:
                    if(!intakeRotation.isBusy()&&!mineralRotation.isBusy() && !mineralExtension.isBusy()){
                        intakePositionState = NOTHING;
                    }

            }

            switch(depositPositionState){
                case NOTHING:
                    if(gamepad1.a){
                        intakeCurrentPosition = intakeIntakePosition;
                        mineralRotationPosition = rotationExtendPosition;
                        depositPositionState = ROTATION1;
                        depositBlocksState = depositingBlocksPositionState.NOTHING;
                        intakePositionState = NOTHING;
                        drivePositionState = drivingPositionState.NOTHING;
                    }
                    break;
                case ROTATION1:
                    if(!mineralRotation.isBusy()&&!intakeRotation.isBusy()&&!mineralExtension.isBusy()){
                        mineralRotationPosition = mineralRotationDumpBallPosition;
                        mineralExtensionPosition = extensionDumpPositionBalls;
                        depositPositionState = depositingPositionState.EXTENSIONINTAKEROTATION;
                    }
                    break;

                case EXTENSIONINTAKEROTATION:
                    if(mineralExtension.getCurrentPosition() > (extensionDumpPositionBalls/4)){
                        intakeCurrentPosition = intakeDumpReadyPosition;
                    }
                    if(!mineralRotation.isBusy()&&!intakeRotation.isBusy()&&!mineralExtension.isBusy()){
                        depositPositionState = depositPositionState.NOTHING;
                    }
                    break;
            }

            switch (depositBlocksState){
                case NOTHING:
                    if(gamepad1.y){
                        intakeCurrentPosition = intakeIntakePosition;
                        mineralRotationPosition = rotationExtendPosition;
                        depositBlocksState = depositingBlocksPositionState.ROTATION1;
                        depositPositionState = depositingPositionState.NOTHING;
                        intakePositionState = NOTHING;
                        drivePositionState = drivingPositionState.NOTHING;
                    }
                    break;
                case ROTATION1:
                    if(!mineralRotation.isBusy()&&!intakeRotation.isBusy()&&!mineralExtension.isBusy()){
                        mineralRotationPosition = mineralRotationDumpBallPosition;
                        mineralExtensionPosition = extensionDumpPositionBlocks;
                        depositBlocksState = depositingBlocksPositionState.EXTENSIONINTAKEROTATION;
                    }
                    break;

                case EXTENSIONINTAKEROTATION:
                    if(mineralExtension.getCurrentPosition() > (extensionDumpPositionBalls/4)){
                        intakeCurrentPosition = intakeDumpReadyPositionBlocks;
                    }
                    if(!mineralRotation.isBusy()&&!intakeRotation.isBusy()&&!mineralExtension.isBusy()){
                        depositBlocksState = depositingBlocksPositionState.NOTHING;
                    }
                    break;
            }

            switch(drivePositionState){
                case NOTHING:
                    if(gamepad1.b){
                        depositBlocksState = depositingBlocksPositionState.NOTHING;
                        depositPositionState = depositingPositionState.NOTHING;
                        intakePositionState = NOTHING;
                        intaking = false;
                        mineralRotationPosition = rotationExtendPosition;
                        intakeCurrentPosition = intakeIntakePosition;
                        drivePositionState = drivingPositionState.ROTATION1;
                    }
                    break;

                case ROTATION1:
                    if(mineralRotation.getCurrentPosition() < 775){
                        mineralExtensionPosition = 0;
                    }
                    if(!mineralRotation.isBusy()&&!mineralExtension.isBusy()&&!intakeRotation.isBusy()){
                        mineralRotationPosition = rotationDrivePosition;
                        intakeCurrentPosition = intakeIntakePosition;
                        drivePositionState = drivingPositionState.FINALPOSITION;
                    }
                    break;
                case FINALPOSITION:
                    if(!mineralRotation.isBusy()&&!mineralExtension.isBusy()&&!intakeRotation.isBusy()){
                        drivePositionState = drivingPositionState.NOTHING;
                    }
            }

            telemetry.addLine("Press Gamepad 1 B to go to Drive Position");
            telemetry.addLine("Press Gamepad 1 X to go to Intake Position");
            telemetry.addLine("Press Gamepad 1 A to go to Ball Deposit Position");
            telemetry.addLine("Press Gamepad 1 Y to go to Block Deposit Position");
            telemetry.addLine("Press Gamepad 1 Up to go to Open Gate");
        }
    }
}

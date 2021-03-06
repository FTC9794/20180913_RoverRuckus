package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

import java.io.File;

import static org.firstinspires.ftc.teamcode.teleop.Master_Teleop_V2_FrameOfReference.depositingPositionState.EXTENSIONINTAKEROTATION;
import static org.firstinspires.ftc.teamcode.teleop.Master_Teleop_V2_FrameOfReference.intakingPositionState.NOTHING;
import static org.firstinspires.ftc.teamcode.teleop.Master_Teleop_V2_FrameOfReference.intakingPositionState.ROTATING;

/**
 * Created by Sarthak on 10/26/2018.
 */
@TeleOp(name = "\uD83C\uDFAE Master Teleop V2 Frame of Reference", group = "Teleop")
@Deprecated
@Disabled
public class Master_Teleop_V2_FrameOfReference extends LinearOpMode {

    /*

    Drivetrain variables

     */
    DcMotor rf, rb, lf, lb;
    Servo phoneServo;
    Servo intakeGate;
    final double GATE_OPEN = 1, GATE_CLOSED = 0.425;
    double[] drivePower = new double[4];
    final double reducedPower = .75, phoneStoredPosition = .5, rotationMinPower = .1;

    IIMU imu;
    BNO055IMU boschIMU;
    boolean fieldCentric;


    /*

    Hang Variables

     */
    DcMotor hang;
    Servo hangStopper;
    DigitalChannel hangLimit;
    int hangCurrentPosition;
    double hangUpPower, hangDownPower;
    final int hangReadyPosition = 2640, hangMaxPosition = 5000, hangLatchPosition = 5675, hangHungPosition = 20, hangAlignPosition = 3330;
    final double hangStopperStoredPosition = 0.5;
    public enum hangState {NOTHING, LATCHING, HANGING};
    hangState currentHangingState = hangState.NOTHING;
    boolean hangReady = false;
    boolean yPressedToggle  = false, yPressed = false, latchReady = false;

    Rev2mDistanceSensor latch_detector;

    public enum intakingPositionState{NOTHING, EXTENDING, ROTATING};
    intakingPositionState intakePositionState = intakingPositionState.NOTHING;
    public enum depositingPositionState{NOTHING, INIT, ROTATION1, EXTENSIONINTAKEROTATION, ROTATION2};
    public enum depositingBlocksPositionState{NOTHING, INIT, ROTATION1, EXTENSIONINTAKEROTATION, ROTATION2};
    public enum drivingPositionState{NOTHING, ROTATION1, FINALPOSITION};
    depositingPositionState depositPositionState = depositingPositionState.NOTHING;
    depositingBlocksPositionState depositBlocksState = depositingBlocksPositionState.NOTHING;
    drivingPositionState drivePositionState = drivingPositionState.NOTHING;

    /*

    Team Marker variables

     */
    Servo teamMarker;
    final double teamMarkerStoredPosition = 1, teamMarkerDepositPosition = 1;

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
    int intakeDumpReadyPosition = 425, intakeDumpReadyPositionBlocks = 270, intakeIntakePosition = 535;
    final double intakeInPower = .73, intakeOutPower = -.73;
    double intakeRotationPower = .5;
    int intakeCurrentPosition;

    double joystickX, joystickY, joystickAngle, joystickMagnitude, robotAngle, motionAngle, motionMagnitude, motionComponentX, motionComponentY, pivot;

    boolean intakePressed = false;
    boolean intaking = false;
    boolean reset = false;

    double mineralRotationMechPower = 0.175;

    File mineralExtensionEncoderPosition = AppUtil.getInstance().getSettingsFile("mineralExtensionEncoderPosition.txt");
    File mineralRotationEncoderPosition = AppUtil.getInstance().getSettingsFile("mineralRotationEncoderPosition.txt");
    File intakeRotationEncoderPosition = AppUtil.getInstance().getSettingsFile("intakeRotationEncoderPosition.txt");

    File autoIMUOffset = AppUtil.getInstance().getSettingsFile("autoAngle.txt");
    double imuOffset = 0;

    ElapsedTime extensionTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        /*

        Drivetrain Initialization

         */
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        phoneServo = hardwareMap.servo.get("scanner");

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

        //Disable field centric driving by default
        fieldCentric = false;

        /*

        Hang Initialization

         */
        hang = hardwareMap.dcMotor.get("hang");
        hangStopper = hardwareMap.servo.get("hang_stopper");
        hangLimit = hardwareMap.digitalChannel.get("hang_limit");

        //hang.setDirection(DcMotorSimple.Direction.REVERSE);

        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*

        team marker initialization

        */
        teamMarker = hardwareMap.servo.get("marker_servo");

        /*

        Mineral rotation and extension initialization

         */
        mineralRotation = hardwareMap.dcMotor.get("mineral_rotation");
        mineralExtension = hardwareMap.dcMotor.get("mineral_extension");
        rotationLimit = hardwareMap.digitalChannel.get("rotation_limit");

        mineralRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mineralExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int mineralExtensionPosition, mineralRotationPosition;
        double mineralRotationPower;

        mineralExtensionPosition = 0;
        mineralRotationPosition = 0;

        int extensionMaxPosition = 2700, extensionDumpPositionBalls = 1580,
                extensionDumpPositionBlocks = 1700, extensionDrivePosition = 700, extensionRetractPosition = 500,
                rotationExtendPosition = 725, mineralRotationDumpBallPosition = 1175, mineralRotationDumpBlocksPosition = 2140, mineralRotationIncrement = 50,
                rotationMaxPosition = 1200, rotationDrivePosition = 620, rotationIntakePosition = 0;

        final double mineralExtensionPower = 1, turnMultiplier = (1-rotationMinPower)/(-extensionMaxPosition);
        /*

        Mineral Intake initialization

         */
        intake = hardwareMap.crservo.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRotation = hardwareMap.dcMotor.get("intake_rotation");

        latch_detector = (Rev2mDistanceSensor) hardwareMap.get("latch_detector");

        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intakeRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeCurrentPosition = 0;

        intakeGate = hardwareMap.servo.get("intake_gate");
        intakeGate.setPosition(GATE_OPEN);

        telemetry.addData("initialization", "done");
        telemetry.addData("intake rotation position", intakeRotation.getCurrentPosition());
        telemetry.update();

        mineralExtensionPosition = mineralExtension.getCurrentPosition();
        intakeCurrentPosition = intakeRotation.getCurrentPosition();

        //Initialize IMU
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.initialize();
        imuOffset = Double.parseDouble(ReadWriteFile.readFile(autoIMUOffset).trim());
        imu.setOffset(-imuOffset);

        boolean selected = false;
        String hand = "";
        while(!selected && !isStopRequested()){
            telemetry.addData("Gamepad1 DPad Left", "Lefty Mode");
            telemetry.addData("Gamepad1 DPad Right", "Righty Mode");
            telemetry.update();
            if(gamepad1.dpad_left){
                hand = "left";
                selected = true;
            }else if(gamepad1.dpad_right){
                hand = "right";
                selected = true;
            }
        }

        hangCurrentPosition = hang.getCurrentPosition();

        telemetry.addLine("Init Complete");
        telemetry.addData("Extension Position", mineralExtension.getCurrentPosition());
        telemetry.addData("Mineral Rotation Position", mineralRotation.getCurrentPosition());
        telemetry.addData("Intake Rotation", intakeRotation.getCurrentPosition());
        telemetry.addData("Auto Angle Offset", ReadWriteFile.readFile(autoIMUOffset));
        telemetry.addData("Hand Selected", hand);
        telemetry.update();

        waitForStart();

        //initialize servo positions after start is pressed to be legal
        hangStopper.setPosition(hangStopperStoredPosition);
        teamMarker.setPosition(teamMarkerStoredPosition);
        phoneServo.setPosition(phoneStoredPosition);

        while (opModeIsActive()){

            /*

            Drivetrain code

             */

            //Check if still field centric
            if(gamepad1.dpad_down){
                fieldCentric = false;
            }

            //check to reset IMU value
            if(gamepad1.dpad_up){
                imu.setAsZero();
                fieldCentric = true;
            }

            //Get gamepad values
            if(hand.equals("left")){
                joystickY = -gamepad1.left_stick_y;
                joystickX = gamepad1.left_stick_x;
                if(mineralRotation.getCurrentPosition() < 300) {
                    pivot = gamepad1.right_stick_x * (turnMultiplier * mineralExtensionPosition + 1);
                }else{
                    pivot = gamepad1.right_stick_x;
                }
                if(pivot < 0 && pivot > -0.1){
                    pivot = -0.25;
                }else if (pivot > 0 && pivot < 0.1){

                }
            }else{
                joystickY = -gamepad1.right_stick_y;
                joystickX = gamepad1.right_stick_x;
                if(mineralRotation.getCurrentPosition() < 300) {
                    pivot = gamepad1.left_stick_x * (turnMultiplier * mineralExtensionPosition + 1);
                }else{
                    pivot = gamepad1.left_stick_x;
                }
                if(pivot < 0 && pivot > -0.1){
                    pivot = -0.25;
                }else if (pivot > 0 && pivot < 0.1){
                    pivot = 0.25;
                }
            }

            //determine if we are driving field centricly or not
            if(fieldCentric) {
                robotAngle = imu.getZAngle();
            }else{
                robotAngle = 0;
            }

            //calculate the polar vector of the joystick
            joystickAngle = Math.toDegrees(Math.atan2(joystickX, joystickY));
            joystickMagnitude = Math.sqrt(Math.pow(joystickX, 2)+Math.pow(joystickY,2));

            //calculate the motion polar vector
            motionAngle = joystickAngle-robotAngle;
            if(joystickMagnitude>1){
                motionMagnitude = 1;
            }else{
                motionMagnitude = joystickMagnitude;
            }

            //break the motion vector up into it's components
            motionComponentX = motionMagnitude*(Math.sin(Math.toRadians(motionAngle)));
            motionComponentY = motionMagnitude*(Math.cos(Math.toRadians(motionAngle)));

            //determine motor powers
            drivePower[0] = motionComponentY*Math.abs(motionComponentY)-motionComponentX*Math.abs(motionComponentX)-pivot;
            drivePower[1] = motionComponentY*Math.abs(motionComponentY)+motionComponentX*Math.abs(motionComponentX)-pivot;
            drivePower[2] = motionComponentY*Math.abs(motionComponentY)+motionComponentX*Math.abs(motionComponentX)+pivot;
            drivePower[3] = motionComponentY*Math.abs(motionComponentY)-motionComponentX*Math.abs(motionComponentX)+pivot;


            //crop powers to fit in limits
            for(int i=0; i<drivePower.length; i++){
                if(drivePower[i]>1){
                    drivePower[i] = 1;
                }else if(drivePower[i]<-1){
                    drivePower[i] = -1;
                }
            }

            //reduce power if desired
            if(!(gamepad1.right_stick_button||gamepad1.left_stick_button)){
                for(int i=0; i<drivePower.length; i++){
                    drivePower[i] = drivePower[i]*reducedPower;
                }
            }


            //set motor power
            rf.setPower(drivePower[0]);
            rb.setPower(drivePower[1]);
            lf.setPower(drivePower[2]);
            lb.setPower(drivePower[3]);

            if(intakeRotation.getCurrentPosition() < (intakeIntakePosition-435)){
                intakeGate.setPosition(GATE_OPEN);
            }else if(gamepad1.a){
                intakeGate.setPosition(GATE_OPEN);
            }else{
                intakeGate.setPosition(GATE_CLOSED);
            }




            /*

            Hang Code

             */

            hangUpPower = gamepad1.right_trigger;
            hangDownPower = gamepad1.left_trigger;

            if(hangUpPower>0){
                hangCurrentPosition = hang.getCurrentPosition();
                hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //if(hangCurrentPosition>hangMaxPosition){
                  //  hang.setPower(0);
                //}else{
                    hang.setPower(hangUpPower);
                //}
                //hangReady = false;
                currentHangingState = hangState.NOTHING;
            }else if(hangDownPower>0){
                hangCurrentPosition = hang.getCurrentPosition();
                hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(hangLimit.getState()){
                    hang.setPower(-hangDownPower);
                }else{
                    hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hang.setPower(0);
                }
                //hangReady = false;
                latchReady = false;
                currentHangingState = hangState.NOTHING;

            }else{

                if(!hang.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
                    hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                hang.setTargetPosition(hangCurrentPosition);
                hang.setPower(1);
            }

            yPressed = gamepad1.x;
            if(yPressed&&!yPressedToggle&&!hangReady){
                hangCurrentPosition = hangLatchPosition;
                hangReady = true;
            }
            yPressedToggle = yPressed;

            if(gamepad1.y){
                hangCurrentPosition = hangAlignPosition;
                hangReady = false;
            }
            /*if(yPressed&&!yPressedToggle&&!latchReady){
                hangCurrentPosition = hangReadyPosition;
                latchReady = true;
            }else if(yPressed&&!yPressedToggle&&latchReady){
                hangCurrentPosition = hangLatchPosition;
                latchReady = false;
                hangReady = true;
            }*/


            /*

            Mineral Rotation and extension code

             */

//            if(gamepad2.right_bumper){
//                mineralExtensionPosition = 0;
//                intakeCurrentPosition = 0;
//                mineralRotationPosition = 0;
//                drivePositionState = drivingPositionState.NOTHING;
//            }

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

            telemetry.addData("Extension Current Position", mineralExtension.getCurrentPosition());
            telemetry.addData("Extension Target Position", mineralExtensionPosition);
            telemetry.addData("Raw Rotation Position", mineralRotation.getCurrentPosition());
            telemetry.addData("Rotation Target Position ", mineralRotationPosition);
            telemetry.addData("Rotation Intake Position", rotationIntakePosition);

            mineralRotationPower = -gamepad2.right_stick_y;
            if(mineralRotationPower!=0){
                mineralRotationMechPower = 0.75;
                depositBlocksState = depositingBlocksPositionState.NOTHING;
                depositPositionState = depositingPositionState.NOTHING;
                intakePositionState = NOTHING;
                drivePositionState = drivingPositionState.NOTHING;
                if(mineralRotationPower<0&&!rotationLimit.getState()){
                    if(!reset){
                        mineralRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        reset = true;
                    }else{
                        mineralRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        mineralRotationPosition = 0;
                    }
                    //mineralRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //mineralRotation.setPower(0);
                    /*
                    rotationIntakePosition = mineralRotation.getCurrentPosition();
                    rotationDrivePosition = rotationDrivePosition - rotationIntakePosition;
                    mineralRotationDumpBallPosition = mineralRotationDumpBallPosition - rotationIntakePosition;
                    rotationExtendPosition = rotationExtendPosition - rotationIntakePosition;
                    mineralRotationPosition = rotationIntakePosition;
                    */
                }else{
                    reset = false;
                    mineralRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(mineralRotationPower<0){

                        mineralRotationPosition = (int) (mineralRotationPosition-mineralRotationIncrement);
                    }else{
                        if(!(mineralRotationPosition>rotationMaxPosition)){
                            mineralRotationPosition = mineralRotationPosition+mineralRotationIncrement;
                        }

                    }
                    if(mineralRotation.getCurrentPosition() > mineralRotationPosition){
                        mineralRotation.setPower(mineralRotationMechPower);
                    }else{
                        mineralRotation.setPower(mineralRotationMechPower);
                    }
                    mineralRotation.setTargetPosition(mineralRotationPosition);

                }

            }else{
                if(!mineralRotation.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
                    mineralRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if(mineralRotation.getCurrentPosition() < rotationExtendPosition-100){
                    mineralRotation.setPower(mineralRotationMechPower);
                }else{
                    mineralRotation.setPower(mineralRotationMechPower);
                }

//                if(mineralRotationPosition <= mineralRotationDumpBallPosition - 50){
//                    if(mineralRotation.getCurrentPosition() > mineralRotationPosition && mineralRotation.isBusy()){
//                        mineralRotation.setPower(0.175);
//                    }else{
//                        mineralRotation.setPower(0.175);
//                    }
//                }else{
//                    mineralRotation.setPower(0);
//                }

                mineralRotation.setTargetPosition(mineralRotationPosition);
            }

            /*

            Mineral Intake Code

             */

            if(gamepad2.left_bumper){
                intakeIntakePosition = intakeRotation.getCurrentPosition();
                intakeDumpReadyPosition = intakeIntakePosition - 125;
                intakeDumpReadyPositionBlocks = intakeIntakePosition - 215;
            }

            if(gamepad2.left_trigger>.01){
                intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeCurrentPosition = intakeRotation.getCurrentPosition();
                intakeRotation.setPower(intakeRotationPower);
                depositBlocksState = depositingBlocksPositionState.NOTHING;
                depositPositionState = depositingPositionState.NOTHING;
                intakePositionState = NOTHING;
                drivePositionState = drivingPositionState.NOTHING;
            }else if(gamepad2.right_trigger>.01){
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

            /*if(gamepad1.right_bumper){
                if(!intakePressed){
                    intake.setPower(intakeInPower);
                }


            }else if(gamepad1.left_bumper){
                intake.setPower(intakeOutPower);

            }else{
                intake.setPower(0);
            }*/

             /*

             Team Marker code

             */
            if(gamepad1.y){
                teamMarker.setPosition(teamMarkerDepositPosition);
            }else{
                teamMarker.setPosition(teamMarkerStoredPosition);
            }

            telemetry.addData("Intake Rotation Position", intakeRotation.getCurrentPosition());
            telemetry.addData("Intake Intake Position", intakeIntakePosition);
            telemetry.addData("Hang Current Position", hang.getCurrentPosition());
            telemetry.update();



            /*

            State Machines

             */

            switch(intakePositionState){
                case NOTHING:
                    if(gamepad2.a){
                        intakePositionState = ROTATING;
                        intakeCurrentPosition = intakeIntakePosition;
                        mineralRotationPosition = rotationIntakePosition;
                        mineralRotationMechPower = 0.8;
                        if(mineralExtension.getCurrentPosition() > 1200){
                            mineralExtensionPosition = 1200;
                        }
                        depositBlocksState = depositingBlocksPositionState.NOTHING;
                        depositPositionState = depositingPositionState.NOTHING;
                        drivePositionState = drivingPositionState.NOTHING;
                        intaking = true;
                    }
                    break;

                case ROTATING:
                    if(!rotationLimit.getState()){
                        mineralRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        mineralRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        mineralRotation.setTargetPosition(0);
                        intakePositionState = NOTHING;
                        mineralRotationMechPower = 0.25;
                    }else{
                        mineralRotationPosition -= 50;
                    }
            }

            switch(depositPositionState){
                case NOTHING:
                    if(gamepad2.x){
                        intaking = false;
                        intakeCurrentPosition = intakeDumpReadyPosition;
                        if(mineralRotation.getCurrentPosition() < 100) {
                            mineralExtensionPosition = extensionRetractPosition;
                        }else{
                            mineralRotationPosition = mineralRotationDumpBallPosition;
                            mineralExtensionPosition = extensionDumpPositionBalls;
                            mineralRotationMechPower = 0.8;
                        }

                        depositPositionState = EXTENSIONINTAKEROTATION;
                        depositBlocksState = depositingBlocksPositionState.NOTHING;
                        intakePositionState = NOTHING;
                        drivePositionState = drivingPositionState.NOTHING;
                    }
                    break;
                case ROTATION1:
                    if(mineralRotation.getCurrentPosition() > rotationDrivePosition){
                        mineralExtensionPosition = extensionDumpPositionBalls;
                    }
                    if(!mineralRotation.isBusy()&&!intakeRotation.isBusy()&&!mineralExtension.isBusy()){
                        depositPositionState = depositPositionState.NOTHING;
                    }
                    break;

                case EXTENSIONINTAKEROTATION:
                    if(Math.abs(mineralExtension.getCurrentPosition() - extensionRetractPosition) < 100 && mineralRotation.getCurrentPosition() < 200){
                        mineralRotationPosition = mineralRotationDumpBallPosition;
                        mineralRotationMechPower = 0.8;
                        depositPositionState = depositPositionState.ROTATION1;
                    }
                    break;
            }

            switch (depositBlocksState){
                case NOTHING:
                    if(gamepad2.b){
                        intakeCurrentPosition = intakeIntakePosition;
                        mineralRotationPosition = rotationExtendPosition;
                        mineralRotationMechPower = 0.5;
                        depositBlocksState = depositingBlocksPositionState.ROTATION1;
                        depositPositionState = depositingPositionState.NOTHING;
                        intakePositionState = NOTHING;
                        drivePositionState = drivingPositionState.NOTHING;
                    }
                    break;
                case ROTATION1:
                    if(mineralRotation.getCurrentPosition() < 750 && mineralRotation.getCurrentPosition() > 600){
                        mineralExtensionPosition = extensionDumpPositionBlocks;
                        mineralRotationMechPower = 0.2;
                        depositBlocksState = depositingBlocksPositionState.EXTENSIONINTAKEROTATION;
                    }
                    break;

                case EXTENSIONINTAKEROTATION:
                    if(mineralExtension.getCurrentPosition() > (extensionDumpPositionBalls/4)){
                        mineralRotationPosition = mineralRotationDumpBallPosition;
                        intakeCurrentPosition = intakeDumpReadyPositionBlocks;
                    }
                    if(!mineralRotation.isBusy()&&!intakeRotation.isBusy()&&!mineralExtension.isBusy()){
                        depositBlocksState = depositingBlocksPositionState.NOTHING;
                    }
                    break;
            }

            if(gamepad2.right_bumper && mineralRotation.getCurrentPosition() < rotationDrivePosition){
                depositBlocksState = depositingBlocksPositionState.NOTHING;
                depositPositionState = depositingPositionState.NOTHING;
                intakePositionState = NOTHING;
                drivePositionState = drivePositionState.NOTHING;
                intakeCurrentPosition = intakeDumpReadyPosition;
                intaking = false;
                if(mineralExtension.getCurrentPosition() > 1200) {
                    mineralExtensionPosition = 1200;
                }
                mineralRotationPosition = 390;
            }

            switch(drivePositionState){
                case NOTHING:
                    if(gamepad2.y){
                        depositBlocksState = depositingBlocksPositionState.NOTHING;
                        depositPositionState = depositingPositionState.NOTHING;
                        intakePositionState = NOTHING;
                        intaking = false;
                        mineralRotationPosition = rotationDrivePosition;
                        if(mineralRotation.getCurrentPosition() < rotationExtendPosition){
                            mineralRotationMechPower = 0.5;
                        }else{
                            mineralRotationMechPower = 0.35;
                        }
                        if(mineralExtension.getCurrentPosition() > extensionDumpPositionBalls){
                            mineralExtensionPosition = extensionDumpPositionBalls;
                        }
                        intakeCurrentPosition = intakeDumpReadyPosition;
                        drivePositionState = drivingPositionState.ROTATION1;
                    }
                    break;

                case ROTATION1:
                    if(mineralRotation.getCurrentPosition() < 925){
                        mineralExtensionPosition = extensionDrivePosition;
                    }
                    if(!mineralRotation.isBusy()&&!mineralExtension.isBusy()){
                        mineralRotationPosition = rotationDrivePosition;
                        intakeCurrentPosition = intakeDumpReadyPosition;
                        drivePositionState = drivingPositionState.FINALPOSITION;
                    }
                    break;
                case FINALPOSITION:
                    if(!mineralRotation.isBusy()&&!mineralExtension.isBusy()){
                        mineralRotationPosition = rotationDrivePosition;
                        mineralRotationMechPower = 0.2;
                        drivePositionState = drivingPositionState.NOTHING;
                    }
            }


            /*
            Hang State Machine
             */
            switch(currentHangingState){
                case NOTHING:
                    if(gamepad1.b&&hangReady){
                        currentHangingState = hangState.HANGING;
                        hangCurrentPosition = 0;
                    }
                    break;
                case HANGING:
                    if(latch_detector.getDistance(DistanceUnit.CM) > hangHungPosition){
                        currentHangingState = hangState.NOTHING;
                        hangCurrentPosition = hang.getCurrentPosition();
                    }
            }

            ReadWriteFile.writeFile(autoIMUOffset, String.valueOf(robotAngle));

        }
    }
}

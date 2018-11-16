package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sarthak on 10/26/2018.
 */
@TeleOp(name = "Master Teleop", group = "Teleop")
public class Master_Teleop extends LinearOpMode {

    /*

    Drivetrain variables

     */
    DcMotor rf, rb, lf, lb;
    Servo phoneServo;
    double[] drivePower = new double[4];
    final double reducedPower = .5, phoneStoredPosition = .5;


    /*

    Hang Variables

     */
    DcMotor hang;
    Servo hangStopper;
    DigitalChannel hangLimit;
    int hangCurrentPosition;
    double hangUpPower, hangDownPower;
    final int hangReadyPosition = 6000, hangMaxPosition = 9600, hangHungPosition = 0;
    final double hangStopperStoredPosition = 1;
    public enum hangState {NOTHING, LATCHING, HANGING};
    hangState currentHangingState = hangState.NOTHING;


    /*

    Team Marker variables

     */
    Servo teamMarker;
    final double teamMarkerStoredPosition = 0, teamMarkerDepositPosition = 1;

    /*

    Mineral Mechanism rotation and extension variables

     */
    DcMotor mineralRotation, mineralExtension;
    DigitalChannel rotationLimit;
    final int extensionMaxPosition = 2700, rotationUpPosition = 500, mineralRotationIncriment = 3, rotationMaxPosition = 1000;
    final double mineralExtensionPower = .5;
    int mineralExtensionPosition, mineralRotationPosition;
    double mineralRotationPower;

    /*

    Intake Mechanism variables

     */
    DcMotor intakeRotation;
    CRServo intake;
    final int intakeDumpPosition = 0, intakeIntakePosition = 0;
    final double intakeInPower = .73, intakeOutPower = -.73;
    double intakeRotationPower = .5;
    int intakeCurrentPosition;

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

        /*

        Hang Initialization

         */
        hang = hardwareMap.dcMotor.get("hang");
        hangStopper = hardwareMap.servo.get("hang_stopper");
        hangLimit = hardwareMap.digitalChannel.get("hang_limit");

        hang.setDirection(DcMotorSimple.Direction.REVERSE);

        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        mineralExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineralExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mineralExtensionPosition = 0;
        mineralRotationPosition = 0;
        /*

        Mineral Intake initialization

         */
        intake = hardwareMap.crservo.get("intake");
        intakeRotation = hardwareMap.dcMotor.get("intake_rotation");

        intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeCurrentPosition = 0;

        telemetry.addData("initialization", "done");
        telemetry.addData("intake rotation position", intakeRotation.getCurrentPosition());
        telemetry.update();

        waitForStart();

        //initialize servo positions after start is pressed to be legal
        hangStopper.setPosition(hangStopperStoredPosition);
        teamMarker.setPosition(teamMarkerStoredPosition);
        phoneServo.setPosition(phoneStoredPosition);

        while (opModeIsActive()){

            /*

            Drivetrain code
d
             */

            //Get gamepad values
            double pitch = -gamepad1.left_stick_y;
            double roll = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;

            drivePower[0] = pitch-roll-pivot;
            drivePower[1] = pitch+roll-pivot;
            drivePower[2] = pitch+roll+pivot;
            drivePower[3] = pitch-roll+pivot;

            for(int i=0; i<drivePower.length; i++){
                if(drivePower[i]>1){
                    drivePower[i] = 1;
                }else if(drivePower[i]<-1){
                    drivePower[i] = -1;
                }
            }
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


            /*

            Hang Code

             */

            hangUpPower = gamepad2.right_trigger;
            hangDownPower = gamepad2.left_trigger;


            if(hangUpPower>0){
                hangCurrentPosition = hang.getCurrentPosition();
                hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(hangCurrentPosition>hangMaxPosition){
                    hang.setPower(0);
                }else{
                    hang.setPower(hangUpPower);
                }

            }else if(hangDownPower>0){
                hangCurrentPosition = hang.getCurrentPosition();
                hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(hangLimit.getState()){
                    hang.setPower(-hangDownPower);
                }else{
                    hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hang.setPower(0);
                }


            }else{

                if(!hang.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
                    hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hang.setTargetPosition(hangCurrentPosition);
                }
                hang.setPower(1);
            }

            if(gamepad2.y){
                hang.setTargetPosition(hangReadyPosition);
            }

            telemetry.addData("hang limit", hangLimit.getState());

            /*

            Mineral Rotation and extension code

             */

            if(gamepad2.dpad_up){
                mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mineralExtensionPosition = mineralExtension.getCurrentPosition();

                if(mineralExtensionPosition>extensionMaxPosition){
                    mineralExtension.setPower(0);
                }else{
                    mineralExtension.setPower(mineralExtensionPower);
                }

            }else if(gamepad2.dpad_down){
                mineralExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mineralExtensionPosition = mineralExtension.getCurrentPosition();
                if(mineralExtensionPosition>0){
                    mineralExtension.setPower(-mineralExtensionPower);
                }else{
                    mineralExtension.setPower(0);
                }

            }else{
                if(mineralExtension.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
                    mineralExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mineralExtension.setTargetPosition(mineralExtensionPosition);
                }
                mineralExtension.setPower(1);

            }


            mineralRotationPower = -gamepad2.right_stick_y;
            if(mineralRotationPower!=0){
                if(mineralRotationPower<0&&!rotationLimit.getState()){
                    mineralRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mineralRotation.setPower(0);
                    mineralRotationPosition = 0;

                }else{
                    mineralRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(mineralRotationPower<0){

                        mineralRotationPosition = (int) (mineralRotationPosition-mineralRotationIncriment);
                    }else{
                        if(!(mineralRotationPosition>rotationMaxPosition)){
                            mineralRotationPosition = mineralRotationPosition+mineralRotationIncriment;
                        }

                    }
                    mineralRotation.setTargetPosition(mineralRotationPosition);

                }

            }else{
                if(!mineralRotation.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
                    mineralRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mineralRotation.setTargetPosition(mineralRotationPosition);
                }
                mineralRotation.setPower(1);
            }

            if(gamepad2.right_bumper){
                mineralRotation.setTargetPosition(rotationUpPosition);
                mineralRotationPosition = rotationUpPosition;
            }

            /*

            Mineral Intake Code

             */

            if(gamepad1.left_trigger>.01){
                intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeCurrentPosition = intakeRotation.getCurrentPosition();
                intakeRotation.setPower(intakeRotationPower);
            }else if(gamepad1.right_trigger>.01){
                intakeRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeCurrentPosition = intakeRotation.getCurrentPosition();
                intakeRotation.setPower(-intakeRotationPower);
            }else{
                if(intakeRotation.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
                    intakeRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeRotation.setTargetPosition(intakeCurrentPosition);
                }
                intakeRotation.setPower(1);
            }



            if(gamepad1.right_bumper){
                intake.setPower(intakeInPower);

            }else if(gamepad1.left_bumper){
                intake.setPower(intakeOutPower);

            }else{
                intake.setPower(0);
            }

             /*

             Team Marker code

             */
            if(gamepad2.b){
                teamMarker.setPosition(teamMarkerDepositPosition);
            }else{
                teamMarker.setPosition(teamMarkerStoredPosition);
            }


            telemetry.update();
        }
    }
}

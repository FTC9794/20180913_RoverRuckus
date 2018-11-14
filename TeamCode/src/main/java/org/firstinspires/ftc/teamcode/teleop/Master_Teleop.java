package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sarthak on 10/26/2018.
 */
@TeleOp(name = "Master Teleop", group = "Teleop")
public class Master_Teleop extends LinearOpMode {
    DcMotor rf, rb, lf, lb, hang;
    Servo hangStopper, teamMarker;
    double[] drivePower = new double[4];

    final int hangReadyPosition = 6000, hangHighPosition = 0, hangHungPosition = 0;
    final double reducedPower = .5;
    final double hangStopperStoredPosition = 1;
    final double teamMarkerStoredPosition = 0, teamMarkerDepositPosition = 1;

    int hangCurrentPosition;
    double hangUpPower, hangDownPower;

    public enum hangState {NOTHING, LATCHING, HANGING};

    hangState currentHangingState;

    @Override
    public void runOpMode() throws InterruptedException {

        /*

        Drivetrain Initialization

         */
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*

        Hang Initialization

         */
        hang = hardwareMap.dcMotor.get("hang");
        hangStopper = hardwareMap.servo.get("hang_stopper");

        hang.setDirection(DcMotorSimple.Direction.REVERSE);

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*

        team marker initialization

        */
        teamMarker = hardwareMap.servo.get("marker_servo");
        waitForStart();

        //initialize servo positions after start is pressed to be legal
        hangStopper.setPosition(hangStopperStoredPosition);
        teamMarker.setPosition(teamMarkerStoredPosition);

        while (opModeIsActive()){

            /*

            Drivetrain code

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
            if(gamepad1.right_stick_button||gamepad1.left_stick_button){
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
                hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hang.setPower(hangUpPower);
                hangCurrentPosition = hang.getCurrentPosition();
            }else if(hangDownPower>0){
                hang.setPower(-hangDownPower);
                hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hangCurrentPosition = hang.getCurrentPosition();
            }else{

                if(hang.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
                    hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hang.setTargetPosition(hangCurrentPosition);
                }

                if(gamepad2.y){
                    hang.setTargetPosition(hangReadyPosition);
                }
                hang.setPower(1);
            }

            /*

            Mineral Rotation and extension code

             */

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

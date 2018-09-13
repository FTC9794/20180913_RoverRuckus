package org.firstinspires.ftc.teamcode.sample_test;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GenericDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.directional.TankDrive2W;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.ITeamMarker;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.ServoArmDrop;
import org.opencv.core.Point;
import org.opencv.core.Size;

import java.util.ArrayList;

/**
 * Created by Sarthak on 9/11/2018.
 */
@Autonomous(name = "Search for Gold Mineral", group = "Test")
public class SearchForGoldMineral extends LinearOpMode{

    IDrivetrain drive;
    DcMotor right, left;
    ArrayList motors;

    IIMU imu;
    BNO055IMU boschIMU;

    ITeamMarker teamMarker;
    Servo teamMarkerArm;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime centertimer;

    final double[] DEFAULT_PID = {.025};
    final double COUNTS_PER_INCH = 47.75;

    enum location {
            LEFT, CENTER, RIGHT, UNKNOWN
    };

    location mineralLocation;

    private GenericDetector genericDetector = null;

    @Override
    public void runOpMode() throws InterruptedException {

        setMotorBehaviors();

        //Set up team marker hardware
        teamMarkerArm = hardwareMap.servo.get("marker");
        teamMarker = new ServoArmDrop(teamMarkerArm);
        teamMarker.hold();
        telemetry.addData("Status", "Team Marker Hardware Initialized");

        //Initialize Vision
        genericDetector = new GenericDetector();
        genericDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        genericDetector.colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW);
        genericDetector.debugContours = false;
        genericDetector.minArea = 700;
        genericDetector.perfectRatio = 1.8;
        genericDetector.stretch = true;
        genericDetector.stretchKernal = new Size(2,50);
        genericDetector.enable();
        centertimer = new ElapsedTime();

        telemetry.addData("Status", "Vision Initialized.");

        //Initialize IMU
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.initialize();
        imu.setOffset(0);
        telemetry.addData("Status", "IMU Instantiated");
        telemetry.update();

        //Setup Drivetrain Subsystem
        drive = new TankDrive2W(motors, imu, telemetry);
        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();


        //Get block in frame
        scanBlock(45);

        //Center block in frame
        centerBlock(1000);

        //Disable detector
        genericDetector.disable();

        double driveAngle = imu.getZAngle();

        drive.resetEncoders();
        while(drive.move(drive.getEncoderDistance(), 36*COUNTS_PER_INCH, 16*COUNTS_PER_INCH, 0, 36*COUNTS_PER_INCH, 0.3,
                0.2, 0, DEFAULT_PID, driveAngle, 50, 250) && opModeIsActive());
        drive.stop();

        if(driveAngle < -13){
            mineralLocation = location.LEFT;
        }else if (Math.abs(driveAngle) <= 10){
            mineralLocation = location.CENTER;
        }else if (driveAngle > 13){
            mineralLocation = location.RIGHT;
        }else{
            mineralLocation = location.UNKNOWN;
        }

        //Move to Alliance Depot and place team marker
        switch(mineralLocation){
            case CENTER:
                drive.resetEncoders();
                while(drive.move(drive.getEncoderDistance(), 36*COUNTS_PER_INCH, 16*COUNTS_PER_INCH, 0, 36*COUNTS_PER_INCH, 0.6,
                        0.25, 0, DEFAULT_PID, driveAngle, 50, 250) && opModeIsActive());
                drive.stop();

                //Deposit team marker
                teamMarker.drop();
                break;

            case LEFT:
                while(drive.pivot(75, 30, 0.4, 0.2, 500, 5, Direction.FASTEST) && opModeIsActive());
                drive.stop();

                drive.resetEncoders();
                while(drive.move(drive.getEncoderDistance(), 36*COUNTS_PER_INCH, 16*COUNTS_PER_INCH, 0, 30*COUNTS_PER_INCH, 0.6,
                        0.25, 0, DEFAULT_PID, 25, 50, 250) && opModeIsActive());
                drive.stop();

                //Deposit team marker
                teamMarker.drop();
                break;

            case RIGHT:
                while(drive.pivot(-75, 0, 0.4, 0.2, 500, 5, Direction.FASTEST) && opModeIsActive());
                drive.stop();

                drive.resetEncoders();
                while(drive.move(drive.getEncoderDistance(), 36*COUNTS_PER_INCH, 16*COUNTS_PER_INCH, 0, 30*COUNTS_PER_INCH, 0.6,
                        0.25, 0, DEFAULT_PID, -25, 50, 250) && opModeIsActive());
                drive.stop();

                //Deposit team marker
                teamMarker.drop();
                break;

            case UNKNOWN:
                //Do nothing
                break;
        }

        while(opModeIsActive()){
            drive.stop();
            telemetry.addData("Status", "Finished");
            telemetry.addData("Drive Angle", driveAngle);
            telemetry.update();
        }


    }

    private void scanBlock(double scanRadius){
        boolean blockFound = genericDetector.getFound();

        while(drive.pivot(-scanRadius, -30, 0.2, 0.15, 500, 1, Direction.FASTEST) && !blockFound &&opModeIsActive()){
            if(genericDetector.getFound()){
                blockFound = true;
            }
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.addData("Block Detected", genericDetector.getFound());
            telemetry.update();
        }

        drive.stop();

        if(!blockFound){
            while(drive.pivot(scanRadius, -30, 0.2, 0.15, 500, 1, Direction.FASTEST) && !blockFound &&opModeIsActive()){
                if(genericDetector.getFound()){
                    blockFound = true;
                }
                telemetry.addData("IMU Angle", imu.getZAngle());
                telemetry.addData("Block Detected", genericDetector.getFound());
                telemetry.update();
            }
        }
    }

    private void centerBlock(double correctionTime){
        centertimer.reset();

        Point blockLocation = null;

        boolean centered = false;
        boolean centertimerstarted = false;
        boolean centertimerfinished = false;
        double x = 0;
        double targetX = 275;
        while(opModeIsActive() && !centertimerfinished){
            blockLocation = genericDetector.getLocation();
            if(blockLocation != null){
                x = blockLocation.x;
            }

            if(Math.abs(x - targetX) <= 25){
                right.setPower(0);
                left.setPower(0);
                if(centered==false){
                    centertimer.reset();
                    centertimerstarted = true;
                    centertimerfinished = false;
                }else if(centertimerstarted = true){
                    centertimerfinished = false;
                    if(centertimer.milliseconds()>correctionTime){
                        centertimerfinished = true;
                    }
                }
                centered = true;
            }else if(((x-targetX)/1000)<.175 && ((x-targetX)/1000)>0) {
                right.setPower(-.175);
                left.setPower(.175);
            }else if((((x-targetX)/1000)>-.15 && ((x-targetX)/1000)<0)) {
                right.setPower(.175);
                left.setPower(-.175);
            }else{
                right.setPower(-(x-targetX)/1000);
                left.setPower((x-targetX)/1000);
                telemetry.addData("Status", "Centering Block in Frame");
                telemetry.addData("X Value", x);
                telemetry.addData("X Difference", x - targetX);
            }


            telemetry.update();
        }
        drive.stop();
    }

    private void setMotorBehaviors(){
        //Hardware Map
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");
        //Set motor behaviors
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Set Zero Power Behavior
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Reverse Right Motor
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new ArrayList<>();
        motors.add(right);
        motors.add(left);
        telemetry.addData("Status", "Motor Hardware Initialized");
        telemetry.update();
    }


}

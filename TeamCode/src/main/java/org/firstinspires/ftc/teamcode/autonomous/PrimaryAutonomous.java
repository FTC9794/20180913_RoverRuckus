package org.firstinspires.ftc.teamcode.autonomous;

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
@Autonomous(name = "Primary Auto", group = "Autonomous")
public class PrimaryAutonomous extends LinearOpMode{

    IDrivetrain drive;
    DcMotor right, left;
    ArrayList motors;

    IIMU imu;
    BNO055IMU boschIMU;

    ITeamMarker teamMarker;
    Servo teamMarkerServo;

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
        teamMarkerServo = hardwareMap.servo.get("marker");
        teamMarker = new ServoArmDrop(teamMarkerServo);
        teamMarker.hold();
        telemetry.addData("Status", "Team Marker Hardware Initialized");

        //Initialize Vision and Start Detector
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
        boolean blockFound = scanBlock(60);

        if(!blockFound){
            while (opModeIsActive());
        }
        //Center block in frame
        centerBlock(1000);

        //Disable detector
        genericDetector.disable();

        double driveAngle = imu.getZAngle();

        if(driveAngle < -13){
            mineralLocation = location.LEFT;
        }else if (Math.abs(driveAngle) <= 10){
            mineralLocation = location.CENTER;
        }else if (driveAngle > 13){
            mineralLocation = location.RIGHT;
        }else{
            mineralLocation = location.UNKNOWN;
        }

        wait(750, runtime);

        //Move to Alliance Depot and place team marker
        switch(mineralLocation) {
            case CENTER:

                //Drive to knock off gold mineral
                drive.resetEncoders();
                while(drive.move(drive.getEncoderDistance(), 32*COUNTS_PER_INCH, 21*COUNTS_PER_INCH, 0, 36*COUNTS_PER_INCH, 0.3,
                        0.2, 0, DEFAULT_PID, driveAngle, 50, 250) && opModeIsActive());
                drive.stop();

                drive.resetEncoders();
                while (drive.move(drive.getEncoderDistance(), 20 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0, 20 * COUNTS_PER_INCH, 0.3,
                        0.25, 0, DEFAULT_PID, driveAngle, 50, 0) && opModeIsActive()) ;
                drive.stop();
                runtime.reset();
                wait(750, runtime);

                //Deposit team marker
                teamMarker.drop();
                runtime.reset();
                wait(750, runtime);

                drive.resetEncoders();
                while (drive.move(drive.getEncoderDistance(), 4 * COUNTS_PER_INCH, 2 * COUNTS_PER_INCH, 0, 2 * COUNTS_PER_INCH, 0.2,
                        0.2, 180, DEFAULT_PID, driveAngle, 50, 250) && opModeIsActive()) ;
                drive.stop();

                wait(750, runtime);

                while(drive.pivot(-95, -45, 0.3, 0.15, 500, 5, Direction.FASTEST) && opModeIsActive());
                drive.stop();

                drive.resetEncoders();
                runtime.reset();
                while (drive.move(drive.getEncoderDistance(), 20 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 0, 16 * COUNTS_PER_INCH, 0.2,
                        0.2, 0, DEFAULT_PID, -95, 50, 250) && opModeIsActive() && runtime.milliseconds() < 3000) ;
                drive.stop();

                while(drive.pivot(-127, -115, 0.3, 0.15, 500, 5, Direction.FASTEST) && opModeIsActive());

                wait(750, runtime);

                drive.resetEncoders();
                while (drive.move(drive.getEncoderDistance(), 12 * COUNTS_PER_INCH, 12 * COUNTS_PER_INCH, 0, 12 * COUNTS_PER_INCH, 0.4,
                        0.3, 0, DEFAULT_PID, -127, 50, 250) && opModeIsActive()) ;
                while (drive.move(drive.getEncoderDistance(), 24 * COUNTS_PER_INCH, 12 * COUNTS_PER_INCH, 0, 24 * COUNTS_PER_INCH, 0.4,
                        0.3, 0, DEFAULT_PID, -135, 50, 250) && opModeIsActive()) ;
                drive.stop();

                break;

            case LEFT:

                //Drive to knock off gold mineral
                drive.resetEncoders();
                while(drive.move(drive.getEncoderDistance(), 44*COUNTS_PER_INCH, 22*COUNTS_PER_INCH, 0, 44*COUNTS_PER_INCH, 0.3,
                        0.2, 0, DEFAULT_PID, driveAngle, 50, 250) && opModeIsActive());
                drive.stop();

                int pivotAngleLeft = 40;
                while (drive.pivot(pivotAngleLeft, 30, 0.3, 0.15, 500, 5, Direction.FASTEST) && opModeIsActive())
                    ;
                drive.stop();

                wait(750, runtime);

                drive.resetEncoders();
                while (drive.move(drive.getEncoderDistance(), 19 * COUNTS_PER_INCH, 16 * COUNTS_PER_INCH, 0, 19 * COUNTS_PER_INCH, 0.3,
                        0.2, 0, DEFAULT_PID, pivotAngleLeft, 50, 0) && opModeIsActive()) ;
                drive.stop();

                wait(750, runtime);

                //Deposit team marker
                teamMarker.drop();
                runtime.reset();
                wait(750, runtime);

                drive.resetEncoders();
                while (drive.move(drive.getEncoderDistance(), 4 * COUNTS_PER_INCH, 2 * COUNTS_PER_INCH, 0, 2 * COUNTS_PER_INCH, 0.2,
                        0.2, 180, DEFAULT_PID, pivotAngleLeft, 50, 250) && opModeIsActive()) ;
                drive.stop();

                wait(750, runtime);

                while(drive.pivot(45, -90, 0.4, 0.2, 500, 5, Direction.FASTEST) && opModeIsActive());

                drive.resetEncoders();
                while (drive.move(drive.getEncoderDistance(), 24 * COUNTS_PER_INCH, 12 * COUNTS_PER_INCH, 0, 24 * COUNTS_PER_INCH, 0.4,
                        0.3, 0, DEFAULT_PID, 45, 50, 250) && opModeIsActive()) ;
                drive.stop();

                break;

            case RIGHT:

                //Drive to knock off gold mineral
                drive.resetEncoders();
                while(drive.move(drive.getEncoderDistance(), 42*COUNTS_PER_INCH, 21*COUNTS_PER_INCH, 0, 42*COUNTS_PER_INCH, 0.3,
                        0.2, 0, DEFAULT_PID, driveAngle, 50, 250) && opModeIsActive());
                drive.stop();

                int pivotAngleRight = -25;
                while (drive.pivot(pivotAngleRight, 0, 0.3, 0.15, 500, 5, Direction.FASTEST) && opModeIsActive())
                    ;
                drive.stop();

                wait(750, runtime);

                drive.resetEncoders();
                while (drive.move(drive.getEncoderDistance(), 16 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 0, 16 * COUNTS_PER_INCH, 0.3,
                        0.2, 0, DEFAULT_PID, pivotAngleRight, 50, 0) && opModeIsActive()) ;
                drive.stop();

                wait(750, runtime);

                //Deposit team marker
                teamMarker.drop();
                runtime.reset();
                wait(750, runtime);

                drive.resetEncoders();
                while (drive.move(drive.getEncoderDistance(), 4 * COUNTS_PER_INCH, 2 * COUNTS_PER_INCH, 0, 2 * COUNTS_PER_INCH, 0.2,
                        0.2, 180, DEFAULT_PID, pivotAngleRight, 50, 250) && opModeIsActive());
                drive.stop();

                wait(750, runtime);

                while(drive.pivot(-50, 0, 0.3, 0.15, 500, 5, Direction.FASTEST) && opModeIsActive());
                drive.stop();

                drive.resetEncoders();
                while (drive.move(drive.getEncoderDistance(), 12 * COUNTS_PER_INCH, 12 * COUNTS_PER_INCH, 0, 12 * COUNTS_PER_INCH, 0.2,
                        0.2, 180, DEFAULT_PID, -50, 50, 250) && opModeIsActive()) ;
                while (drive.move(drive.getEncoderDistance(), 24 * COUNTS_PER_INCH, 12 * COUNTS_PER_INCH, 0, 24 * COUNTS_PER_INCH, 0.4,
                        0.2, 180, DEFAULT_PID, -50, 50, 250) && opModeIsActive()) ;
                drive.stop();

                break;

            case UNKNOWN:
                //Do nothing
                while (opModeIsActive()) ;
                break;
        }


        while(opModeIsActive()){
            drive.stop();
            telemetry.addData("Status", "Finished");
            telemetry.addData("Drive Angle", driveAngle);
            telemetry.update();
        }


    }

    private boolean scanBlock(double scanRadius){
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

        return blockFound;
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

    private void wait(double milliseconds, ElapsedTime timer){
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < milliseconds);
    }

}

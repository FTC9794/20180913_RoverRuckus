package org.firstinspires.ftc.teamcode.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.subsystems.sampling.GoldMineralDetector;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.ITeamMarker;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.ServoArmDrop;
import org.opencv.core.Point;

import java.util.ArrayList;

/**
 * Created by Sarthak on 9/15/2018.
 * This autonomous program scores the sampling and drives into the crater for a park
 */
@Autonomous(name = "Ri3D Secondary Auto", group = "Autonomous")
@Deprecated
@Disabled
public class Ri3DSecondaryAutonomous extends LinearOpMode {
    //Drivetrain object and hardware
    IDrivetrain drive;
    DcMotor right, left;
    ArrayList motors;

    //IMU object and hardware
    IIMU imu;
    BNO055IMU boschIMU;

    //Team Marker object and hardware
    ITeamMarker teamMarker;
    Servo teamMarkerServo;

    Servo delatch;

    DcMotor intakeArm;

    //Declare OpMode timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime centertimer;

    //Declare constants
    final double[] DEFAULT_PID = {.025};
    final double COUNTS_PER_INCH = 48.75;

    //Define possible mineral locations in enum
    enum location {
        LEFT, CENTER, RIGHT, UNKNOWN
    };

    //Create location object to store the mineral location data
    location mineralLocation;

    //Create detector to be used for the gold mineral
    private GoldMineralDetector genericDetector = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //Init motor hardware map and behaviors
        setMotorBehaviors();

        //Set up team marker hardware map and object implementation
        teamMarkerServo = hardwareMap.servo.get("marker");
        teamMarker = new ServoArmDrop(teamMarkerServo);
        teamMarker.hold();
        telemetry.addData("Status", "Team Marker Hardware Initialized");

        delatch = hardwareMap.servo.get("delatch");
        delatch.setPosition(1);

        //Initialize Vision and Start Detector
        genericDetector = new GoldMineralDetector();
        genericDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        genericDetector.colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW);
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

        //Delatch from lander
        delatch.setPosition(0);

        runtime.reset();
        //Pivot so that the back of the robot faces the crater
        while(drive.pivot(0, 0, 0.2, 0.15, 500, 5, Direction.FASTEST) && opModeIsActive()
                && runtime.milliseconds() < 2000);

        intakeArm.setTargetPosition(1000);
        intakeArm.setPower(1);
        wait(1000, runtime);

        //Get block in frame, scan in 60 degrees in each direction
        boolean blockFound = scanBlock(60);

        //If the block isn't in the frame after the scan, remain still for the rest of auto
        /*if(!blockFound){
            while (opModeIsActive());
        }*/
        //Center block in frame
        centerBlock(1000);

        //Disable detector
        genericDetector.disable();

        //Get the current orientation of the robot to determine the whether the mineral was left, center, or right
        double driveAngle = imu.getZAngle();

        //Determine the mineral location based on the imu angle
        if(driveAngle < -13){
            mineralLocation = location.LEFT;
        }else if (Math.abs(driveAngle) <= 13){
            mineralLocation = location.CENTER;
        }else if (driveAngle > 13){
            mineralLocation = location.RIGHT;
        }else{
            mineralLocation = location.UNKNOWN;
        }

        //Wait 750 milliseconds before next action
        wait(750, runtime);

        //Move to Alliance Depot and place team marker. Take different paths based on mineral location
        switch(mineralLocation) {
            //If the mineral is located in the center
            case CENTER:

                //Drive 32 inches straight to knock off the gold mineral. Maintain the orientation used to center the block in the frame
                drive.resetEncoders();
                while(drive.move(drive.getEncoderDistance(), 32*COUNTS_PER_INCH, 21*COUNTS_PER_INCH, 0, 36*COUNTS_PER_INCH, 0.3,
                        0.2, 180, DEFAULT_PID, driveAngle, 50, 250) && opModeIsActive());
                drive.stop();

                //Drive 20 inches straight, keeping the same orientation, to move to the crater
                drive.resetEncoders();
                while (drive.move(drive.getEncoderDistance(), 20 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0, 20 * COUNTS_PER_INCH, 0.3,
                        0.25, 180, DEFAULT_PID, driveAngle, 50, 0) && opModeIsActive()) ;
                drive.stop();

                break;

            case LEFT:

                //Drive 44 inches to knock off the gold mineral
                drive.resetEncoders();
                while(drive.move(drive.getEncoderDistance(), 34*COUNTS_PER_INCH, 22*COUNTS_PER_INCH, 0, 34*COUNTS_PER_INCH, 0.3,
                        0.2, 180, DEFAULT_PID, driveAngle, 50, 250) && opModeIsActive());
                drive.stop();

                //Pivot to face the crater
                int pivotAngleLeft = 45;
                while (drive.pivot(pivotAngleLeft, 30, 0.3, 0.15, 500, 5, Direction.FASTEST) && opModeIsActive())
                    ;
                drive.stop();

                //Wait 750 milliseconds before the next action
                wait(750, runtime);

                //Drive 19 inches to move the robot into the depot, to get into position to drop the team marker
                drive.resetEncoders();
                while (drive.move(drive.getEncoderDistance(), 19 * COUNTS_PER_INCH, 16 * COUNTS_PER_INCH, 0, 19 * COUNTS_PER_INCH, 0.3,
                        0.2, 180, DEFAULT_PID, pivotAngleLeft, 50, 0) && opModeIsActive()) ;
                drive.stop();

                break;

            //If the mineral is located on the right
            case RIGHT:

                //Drive 42 inches to knock off the gold mineral
                drive.resetEncoders();
                while(drive.move(drive.getEncoderDistance(), 36*COUNTS_PER_INCH, 21*COUNTS_PER_INCH, 0, 36*COUNTS_PER_INCH, 0.3,
                        0.2, 180, DEFAULT_PID, driveAngle, 50, 250) && opModeIsActive());
                drive.stop();

                //Pivot to face the crater (angle = -25 degrees)
                int pivotAngleRight = -45;
                while (drive.pivot(pivotAngleRight, 0, 0.3, 0.15, 500, 5, Direction.FASTEST) && opModeIsActive())
                    ;
                drive.stop();

                //Wait 750 milliseconds before the next action
                wait(750, runtime);

                //Drive into the crater
                drive.resetEncoders();
                while (drive.move(drive.getEncoderDistance(), 30 * COUNTS_PER_INCH, 15 * COUNTS_PER_INCH, 0, 30 * COUNTS_PER_INCH, 0.3,
                        0.2, 180, DEFAULT_PID, pivotAngleRight, 50, 0) && opModeIsActive()) ;
                drive.stop();

                break;

            case UNKNOWN:
                //Do nothing, mineral location is unknown
                while (opModeIsActive()){
                    telemetry.addData("Drive Angle", driveAngle);
                }
                break;
        }


        while(opModeIsActive()){
            drive.stop();
            telemetry.addData("Status", "Finished");
            telemetry.addData("Drive Angle", driveAngle);
            telemetry.update();
        }


    }

    /**
     * Pivot the robot so that the robot can find the gold mineral. The robot wants to get the gold mineral in the camera frame
     * @param scanRadius degrees to pivot, to scan for the mineral in each direction
     * @return true if the block was found, false if the block was not found
     */
    private boolean scanBlock(double scanRadius){
        //Determine if the block is already in the frame
        boolean blockFound = genericDetector.isFound();

        //Pivot to -scanRadius degrees. If the block is in the frame as the robot pivots, the pivot will stop
        while(drive.pivot(-scanRadius, -30, 0.2, 0.15, 500, 1, Direction.FASTEST) && !blockFound &&opModeIsActive()){
            if(genericDetector.isFound()){ //Record whether the gold mineral is in the frame
                blockFound = true;
            }
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.addData("Block Detected", genericDetector.isFound());
            telemetry.update();
        }

        drive.stop();

        //If the block wasn't in the frame after the first pivot, pivot to scanRadius degrees
        if(!blockFound){
            while(drive.pivot(scanRadius, -30, 0.2, 0.15, 500, 1, Direction.FASTEST) && !blockFound &&opModeIsActive()){
                if(genericDetector.isFound()){
                    blockFound = true;
                }
                telemetry.addData("IMU Angle", imu.getZAngle());
                telemetry.addData("Block Detected", genericDetector.isFound());
                telemetry.update();
            }
        }

        //Return whether the block was found after the scan
        return blockFound;
    }

    /**
     * If the block is in the frame, the robot will center the block in the camera frame. This allows the robot to align to the mineral and get into a position
     * to knock the mineral off its spot in autonomous
     * @param correctionTime amound of time to spend correcting the alignment of the robot, once it the mineral is in a certain x coordinate range
     */
    private void centerBlock(double correctionTime){
        //Reset timer
        centertimer.reset();

        //Create point object to keep track of the gold mineral location
        Point blockLocation = null;

        //Create boolean variables to keep track of the phase of centering the mineral in the frame
        boolean centered = false;
        boolean centertimerstarted = false;
        boolean centertimerfinished = false;
        //Variables to keep track of x position
        double x = 0;
        //Target x position to center the gold mineral
        double targetX = 275;

        //Loop to center the block
        while(opModeIsActive() && !centertimerfinished){
            //Get the location of the gold mineral
            blockLocation = genericDetector.getScreenPosition();
            //Extract the x-value from the blockLocation object
            if(blockLocation != null){
                x = blockLocation.x;
            }

            //Check if the gold mineral is within the frame
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
                //Set motor powers to pivot based on block location
            }else if(((x-targetX)/1000)<.175 && ((x-targetX)/1000)>0) {
                right.setPower(.175);
                left.setPower(-.175);
            }else if((((x-targetX)/1000)>-.15 && ((x-targetX)/1000)<0)) {
                right.setPower(-.175);
                left.setPower(.175);
            }else{
                right.setPower((x-targetX)/1000);
                left.setPower(-(x-targetX)/1000);
                telemetry.addData("Status", "Centering Block in Frame");
                telemetry.addData("X Value", x);
                telemetry.addData("X Difference", x - targetX);
            }


            telemetry.update();
        }
        drive.stop();
    }

    /**
     * Setup the hardware map and the motor modes, zero power behaviors, and direction
     */
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

        intakeArm = hardwareMap.dcMotor.get("intake_arm");
        intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Put drive motors in ArrayList to pass into drivetrain object
        motors = new ArrayList<>();
        motors.add(right);
        motors.add(left);
        //Status update
        telemetry.addData("Status", "Motor Hardware Initialized");
        telemetry.update();
    }

    /**
     * Stop all actions for a specified amount of time (in milliseconds)
     * @param milliseconds amount of time to wait
     * @param timer ElapsedTimer object to keep track of the time
     */
    private void wait(double milliseconds, ElapsedTime timer){
        //Reset the timer
        timer.reset();
        //Wait until the time inputted has fully elapsed
        while(opModeIsActive() && timer.milliseconds() < milliseconds);
    }

}

package org.firstinspires.ftc.teamcode.sample_test;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sarthak on 11/12/2018.
 */
@Autonomous(name = "Vision Scanning Test", group ="Test")
public class VisionScanningTest extends LinearOpMode {
    //Define possible mineral locations in enum
    enum location {
        LEFT, CENTER, RIGHT, UNKNOWN
    };

    //Create location object to store the mineral location data
    location mineralLocation;

    //Create detector to be used for the gold mineral
    private GoldDetector genericDetector = null;

    Servo scanner;
    double rightPosition = 0.6;
    double leftPosition = 0.35;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        scanner = hardwareMap.servo.get("scanner");
        scanner.setPosition(0.5);

        genericDetector = new GoldDetector();
        genericDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        genericDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        genericDetector.maxAreaScorer.weight = 0.005;

        genericDetector.ratioScorer.weight = 5;
        genericDetector.ratioScorer.perfectRatio = 1.0;

        genericDetector.enable();

        waitForStart();
        runtime.reset();

        boolean found = genericDetector.isFound();
        if(found && (genericDetector.getScreenPosition().x < 250 && genericDetector.getScreenPosition().y > 40)){
            mineralLocation = location.CENTER;
        }else if (found && genericDetector.getScreenPosition().x > 450 && genericDetector.getScreenPosition().y > 40) {
            mineralLocation = location.RIGHT;
        }
         else{
            while(opModeIsActive() && scanner.getPosition() < rightPosition){
                scanner.setPosition(scanner.getPosition() + 0.001);
                telemetry.addData("Y Position", genericDetector.getScreenPosition().y);
                telemetry.update();
            }
            runtime.reset();
            while(runtime.milliseconds() < 1000 && opModeIsActive()){
                telemetry.addData("Y Position", genericDetector.getScreenPosition().y);
                telemetry.update();
            }

            found = genericDetector.isFound();
            if(found && genericDetector.getScreenPosition().x > 100 && genericDetector.getScreenPosition().y > 40) {
                mineralLocation = location.RIGHT;
            }else {
                mineralLocation = location.LEFT;
            }
        }
        while (opModeIsActive()){
            telemetry.addData("Status", "Program Finished");
            telemetry.addData("Mineral Found", genericDetector.isFound());
            telemetry.addData("Mineral Location", mineralLocation);
            telemetry.addData("X Position", genericDetector.getScreenPosition().x);
            telemetry.addData("Y Position", genericDetector.getScreenPosition().y);
            telemetry.addData("Servo Position", scanner.getPosition());
            telemetry.update();
        }
    }
}

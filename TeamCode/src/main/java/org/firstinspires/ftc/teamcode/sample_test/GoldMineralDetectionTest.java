
package org.firstinspires.ftc.teamcode.sample_test;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Point;


@Autonomous(name="Gold Mineral Detection Test", group="Test")
@Disabled
public class GoldMineralDetectionTest extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    private GoldDetector genericDetector = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");


        genericDetector = new GoldDetector();
        genericDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        genericDetector.downscale = 0.7;

        genericDetector.enable();

        telemetry.addData("Status", "Initialized.");

        waitForStart();
        runtime.reset();

        Point blockLocation = null;

        while(opModeIsActive()){
            /*if(genericDetector.isFound() == true){
                telemetry.addData("Location", genericDetector.getScreenPosition());
                //telemetry.addData("Rect", genericDetector.getRect().toString());
                blockLocation = genericDetector.getScreenPosition();
                if(blockLocation != null) {
                    telemetry.addData("X Value", blockLocation.robotGlobalXPosition);
                    if (blockLocation.robotGlobalXPosition < 100) {
                        telemetry.addData("Position", "Left");
                    } else if (blockLocation.robotGlobalXPosition < 400) {
                        telemetry.addData("Position", "Center");
                    } else if (blockLocation.robotGlobalXPosition > 500) {
                        telemetry.addData("Position", "Right");
                    }
                }else{
                    telemetry.addData("Position", "Unknown");
                }
                telemetry.addData("X Position", genericDetector.getScreenPosition().robotGlobalXPosition);
                telemetry.addData("Y Position", genericDetector.getScreenPosition().robotGlobalYPosition);
            }*/
            telemetry.addData("Is Found", genericDetector.isFound());
            telemetry.addData("Pos", genericDetector.getScreenPosition().toString());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

        genericDetector.disable();

    }

}

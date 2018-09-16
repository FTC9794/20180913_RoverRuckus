
package org.firstinspires.ftc.teamcode.sample_test;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GenericDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.vision.GoldMineralDetector;
import org.opencv.core.Point;
import org.opencv.core.Size;


@Autonomous(name="Gold Mineral Detection Test", group="Test")
public class GoldMineralDetectionTest extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    private GoldMineralDetector genericDetector = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");


        genericDetector = new GoldMineralDetector();
        genericDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        genericDetector.colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW);
        genericDetector.debugContours = false;
        genericDetector.minArea = 700;
        genericDetector.perfectRatio = 1.8;
        genericDetector.stretch = true;
        genericDetector.stretchKernal = new Size(2,30);
        genericDetector.enable();

        telemetry.addData("Status", "Initialized.");

        waitForStart();
        runtime.reset();

        Point blockLocation = null;

        while(opModeIsActive()){
            if(genericDetector.getFound() == true){
                telemetry.addData("Location", genericDetector.getLocation().toString());
                //telemetry.addData("Rect", genericDetector.getRect().toString());
                blockLocation = genericDetector.getLocation();
                if(blockLocation != null) {
                    telemetry.addData("X Value", blockLocation.x);
                    if (blockLocation.x < 100) {
                        telemetry.addData("Position", "Left");
                    } else if (blockLocation.x < 400) {
                        telemetry.addData("Position", "Center");
                    } else if (blockLocation.x > 500) {
                        telemetry.addData("Position", "Right");
                    }
                }else{
                    telemetry.addData("Position", "Unknown");
                }
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

        genericDetector.disable();

    }

}

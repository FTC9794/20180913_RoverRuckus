
package org.firstinspires.ftc.teamcode.sample_test;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.sampling.GoldMineralDetector;
import org.opencv.core.Point;
import org.opencv.core.Size;


@Autonomous(name="Gold Mineral Detection Test", group="Test")
public class GoldMineralDetectionTest extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    private GoldMineralDetector genericDetector = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");


        genericDetector = new GoldMineralDetector();
        genericDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        genericDetector.colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW);

        genericDetector.enable();

        telemetry.addData("Status", "Initialized.");

        waitForStart();
        runtime.reset();

        Point blockLocation = null;

        while(opModeIsActive()){
            if(genericDetector.isFound() == true){
                telemetry.addData("Location", genericDetector.getScreenPosition());
                //telemetry.addData("Rect", genericDetector.getRect().toString());
                blockLocation = genericDetector.getScreenPosition();
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

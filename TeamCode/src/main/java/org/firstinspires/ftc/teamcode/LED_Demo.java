package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Sarthak on 4/18/2019.
 */
@TeleOp (name = "LED Demo", group = "LED")
public class LED_Demo extends LinearOpMode {
    RevBlinkinLedDriver led;

    @Override
    public void runOpMode() throws InterruptedException {
        led = (RevBlinkinLedDriver) hardwareMap.get("led");
        waitForStart();
        while(opModeIsActive()) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
        }
    }
}

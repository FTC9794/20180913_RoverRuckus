package org.firstinspires.ftc.teamcode.subsystems.latching;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sarthak on 9/19/2018.
 */
public class PinRelease implements ILatching {

    Servo release;

    final double RELEASE_POSITION = 0.0;
    final double HOLD_POSITION = 1.0;

    public PinRelease(Servo releaseServo){
        this.release = releaseServo;
    }

    @Override
    public void release() {
        release.setPosition(RELEASE_POSITION);
    }

    @Override
    public void hold() {
        release.setPosition(HOLD_POSITION);
    }

    @Override
    public void hook() {
        //Does nothing. Mechanism doesn't have the capabilities
    }
}

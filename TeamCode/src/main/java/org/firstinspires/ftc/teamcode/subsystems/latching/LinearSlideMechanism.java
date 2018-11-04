package org.firstinspires.ftc.teamcode.subsystems.latching;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sarthak on 11/3/2018.
 */
public class LinearSlideMechanism implements ILatching {

    private DcMotor lift;
    private Servo clamp;

    public LinearSlideMechanism(DcMotor lift, Servo clamp){
        this.lift = lift;
        this.clamp = clamp;
    }

    @Override
    public void release() {

    }

    @Override
    public void hold() {

    }

    @Override
    public void hook() {

    }
}

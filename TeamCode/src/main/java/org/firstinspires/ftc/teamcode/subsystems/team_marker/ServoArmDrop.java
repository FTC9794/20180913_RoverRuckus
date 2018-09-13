package org.firstinspires.ftc.teamcode.subsystems.team_marker;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sarthak on 9/12/2018.
 */
public class ServoArmDrop implements ITeamMarker {

    Servo arm;

    final double DROP_POSITION = 0;
    final double HOLD_POSITION = 0.75;

    public ServoArmDrop(Servo teamMarkerArm){
        this.arm = teamMarkerArm;
    }

    @Override
    public void drop() {
        arm.setPosition(DROP_POSITION);
    }

    @Override
    public void hold() {
        arm.setPosition(HOLD_POSITION);
    }

    public void setArmPosition(double position){
        arm.setPosition(position);
    }
}

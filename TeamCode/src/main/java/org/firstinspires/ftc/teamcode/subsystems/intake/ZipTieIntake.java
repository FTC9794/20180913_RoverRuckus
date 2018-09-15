package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

/**
 * Created by Sarthak on 9/14/2018.
 */
public class ZipTieIntake implements IIntake {

    CRServo intakeServo;

    final double INTAKE_POWER = 1;
    final double OUTTAKE_POWER = -1;

    public ZipTieIntake(CRServo intakeServo){
        this.intakeServo = intakeServo;
    }

    @Override
    public void intake() {
        intakeServo.setPower(INTAKE_POWER);
    }

    @Override
    public void outtake() {
        intakeServo.setPower(OUTTAKE_POWER);
    }

    public void stop(){
        intakeServo.setPower(0);
    }

}

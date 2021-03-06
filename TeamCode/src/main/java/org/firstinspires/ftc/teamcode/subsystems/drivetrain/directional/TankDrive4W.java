package org.firstinspires.ftc.teamcode.subsystems.drivetrain.directional;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

import java.util.List;

/**
 * Created by Sarthak on 8/1/2018.
 */
public class TankDrive4W implements IDrivetrain {

    // /hardware of drivetrain
    private List<DcMotor> motors;
    private IIMU imu;

    private DataLogger data;

    //How much the robot can be off the desired angle at the end of a motion
    private final double END_ANGLE_OFFSET = 5;

    private ElapsedTime pivotTime;
    private ElapsedTime distanceCorrectionTimer;

    private boolean targetReached = false;
    private boolean needsToPivot = false;

    //Variables where last encoder value when reset are stored
    private double rfLastEncoder=0;
    private double rbLastEncoder=0;
    private double lfLastEncoder=0;
    private double lbLastEncoder=0;

    Telemetry telemetry;

    public TankDrive4W(List<DcMotor> motors, IIMU imu, Telemetry telemetry){
        this.motors = motors;
        this.imu = imu;
        this.imu.initialize();
        this.telemetry = telemetry;
        pivotTime = new ElapsedTime();
        distanceCorrectionTimer = new ElapsedTime();
    }

    @Override
    public boolean move(double currentPosition, double targetPosition, double rampDownTargetPosition, double rampUpTargetPosition, double rampDownEnd, double maxPower, double lowPower, double moveAngle, double[] PIDGain, double endOrientationAngle, double allowableDistanceError, double correctionTime) {
        //get the difference between the target position and the current position
        double positionDifference = targetPosition - currentPosition;

        //If the robot is within the window of error for distance movements, stop motors
        if(Math.abs(positionDifference)<=allowableDistanceError){
            this.stop();

            if(!targetReached){
                targetReached = true;
                distanceCorrectionTimer.reset();
                return true;
            }

        }else {

            //Calculate the difference between the target position and the position the robot begins to ramp down
            double rampDownDifference = targetPosition - rampDownTargetPosition;
            double rampDownEndDifference = targetPosition - rampDownEnd;

            //Calculate robot power
            double power;
            if (rampDownEndDifference >= Math.abs(positionDifference)) {
                power = lowPower;
            } else if (rampDownDifference > Math.abs(positionDifference)) { //Find the speed based on the speed of the robot's slope and distance traveled
                power = (Math.abs(positionDifference) - rampDownDifference) * ((maxPower - lowPower) / (rampDownDifference - rampDownEndDifference)) + maxPower;
            } else {
                power = maxPower;
            }

            //Calculate orientation corrections
            //Get the current imu angle
            double currentAngle = imu.getZAngle(endOrientationAngle);

            //Calculate the correction to maintain the robot's orientation
            double pivotCorrection = ((currentAngle - endOrientationAngle) * PIDGain[0]);

            this.setPowerAll(power+pivotCorrection, power+pivotCorrection, power-pivotCorrection, power-pivotCorrection);

        }

        if(targetReached&&distanceCorrectionTimer.milliseconds()>=correctionTime){
            this.stop();
            targetReached = false;
            return false;
        }else{
            return true;
        }
    }

    @Override
    public boolean pivot(double desiredAngle, double rampDownAngle, double maxPower, double minPower, double correctionTime, double correctionAngleError, Direction direction) {
        double currentAngle = imu.getZAngle(desiredAngle);
        double angleDifference = desiredAngle-currentAngle;
        double rampDownDifference = desiredAngle - rampDownAngle;
        double power;

        //calculate power
        if(Math.abs(angleDifference)>Math.abs(rampDownDifference)){
            power = maxPower;
        }else{
            power = (maxPower-minPower)/(Math.abs(rampDownDifference)) * Math.abs(angleDifference) + minPower;
        }
        //turn clockwise or counterclockwise depending on which side of desired angle current angle is
        if(direction== Direction.FASTEST||targetReached){
            if(angleDifference>0){
                this.setPowerAll(-power, -power, power, power);
            }else{
                this.setPowerAll(power, power, -power, -power);
            }
        }else if(direction == Direction.CLOCKWISE){
            this.setPowerAll(-power, -power, power, power);
        }else{
            this.setPowerAll(power, power, -power, -power);
        }


        //determine if the pivoting angle is in the desired range
        if(Math.abs(angleDifference)<correctionAngleError&&!targetReached){
            pivotTime.reset();
            targetReached = true;
        }
        if(targetReached && pivotTime.milliseconds()>=correctionTime){
            targetReached = false;
            this.stop();
            return false;
        }else{
            return true;
        }
    }

    @Override
    public void softResetEncoder() {
        rfLastEncoder = motors.get(0).getCurrentPosition();
        lfLastEncoder = motors.get(1).getCurrentPosition();
        rbLastEncoder = motors.get(2).getCurrentPosition();
        lbLastEncoder = motors.get(3).getCurrentPosition();
    }

    @Override
    public void resetEncoders() {
        for(DcMotor motor:motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        rfLastEncoder = 0;
        rbLastEncoder = 0;
        lfLastEncoder = 0;
        lbLastEncoder = 0;
    }

    @Override
    public double getEncoderDistance() {
        int encoderSum = 0;
        encoderSum += (motors.get(0).getCurrentPosition() - rfLastEncoder);
        encoderSum += (motors.get(1).getCurrentPosition() - rbLastEncoder);
        encoderSum += (motors.get(2).getCurrentPosition() - lfLastEncoder);
        encoderSum += (motors.get(3).getCurrentPosition() - lbLastEncoder);
        return (encoderSum/motors.size());
    }

    @Override
    public void stop() {
        this.setPowerAll(0, 0, 0, 0);
    }

    private void setPowerAll(double rfPower, double rbPower, double lfPower, double lbPower){
        motors.get(0).setPower(rfPower);
        motors.get(1).setPower(rbPower);
        motors.get(2).setPower(lfPower);
        motors.get(3).setPower(lbPower);
    }
}

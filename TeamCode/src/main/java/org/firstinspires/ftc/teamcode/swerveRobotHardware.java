package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Created by Delta Robotics on 10/1/2022.
 */
public class swerveRobotHardware extends LinearOpMode
{
    //drive motors
    public DcMotor right1 = null;
    public DcMotor right2 = null;
    public DcMotor left1 = null;
    public DcMotor left2 = null;

    //odometry encoder objects
    public DcMotor leftEncoder = null;
    public DcMotor rightEncoder = null;
    public DcMotor perpendicularEncoder = null;

    public DcMotor[] odometers = new DcMotor[3];
    public DcMotor[] drive = new DcMotor[4];
    VoltageSensor ControlHub_VoltageSensor = null;

    double power = 0;
    double rightPodPosition = 0;
    double leftPodPosition = 0;
    double aTan = 0;
    double turnPowerRight = 0; //angle of the right stick
    double turnPowerLeft = 0; //angle of the left stick
    double turnEncoder = 0;
    double turnPower = 0;
    double currentAngle = 180;
    double newAngle = 0;
    double rotations = 0;
    double distance = 0;
    double opposite = 0;
    double oppositedistance = 0;
    double finalAngle = 0;
    int wheelDirection = 0;

    double encoderTicksPerDegree = 6.40333;

    //PID general Variables

    public static double GeneralF = 0.001; // = 32767 / maxV      (do not edit from this number)
    public static double GeneralP = 0.0025; // = 0.1 * F           (raise till real's apex touches Var apex)
    public static double GeneralI = 0;// = 0.1 * P           (fine adjustment of P)
    public static double GeneralD = 0; // = 0                     (raise to reduce oscillation)

    double GeneralPIDCurrentTime = 0;
    double GeneralPIDTime = 0;
    double GeneralPIDLastTime = 0;
    double GeneralPIDError = 0;
    double GeneralPIDPreviousError = 0;
    double GeneralPIDTotalError = 0;
    double GeneralPIDMinIntegral = -1.0;
    double GeneralPIDMaxIntegral = 1.0;
    double GeneralPIDMotorPower = 0;

    public swerveRobotHardware(HardwareMap ahwMap)
    {

        //drive motors
        left1  = ahwMap.dcMotor.get("one");
        left2  = ahwMap.dcMotor.get("two");
        right1 = ahwMap.dcMotor.get("three");
        right2 = ahwMap.dcMotor.get("four");

        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right1.setPower(0);
        right2.setPower(0);
        left1.setPower(0);
        left2.setPower(0);


        //odometry init (use the motors objects that the odometers are plugged into)
        leftEncoder = left1;
        rightEncoder = left2;
        perpendicularEncoder = right1;

        odometers[0] = leftEncoder;
        odometers[1] = rightEncoder;
        odometers[2] = perpendicularEncoder;

        drive[0] = right1;
        drive[1] = right2;
        drive[2] = left1;
        drive[3] = left2;

        ControlHub_VoltageSensor = ahwMap.get(VoltageSensor.class, "Control Hub");
    }

    public void swerveDrive(double forwardDrive, double strafeDrive, double heading, double speed){
        swerveCalculations(forwardDrive, strafeDrive, heading);

        right1.setPower((wheelDirection * power * speed) + turnPowerRight - turnPower * wheelDirection);
        right2.setPower((wheelDirection * -power * speed) + turnPowerRight + turnPower * wheelDirection);
        left1.setPower((wheelDirection * power * speed) + turnPowerLeft + turnPower * wheelDirection);
        left2.setPower((wheelDirection * -power * speed) + turnPowerLeft - turnPower * wheelDirection);
    }
    public void swerveCalculations(double forwardDrive, double strafeDrive, double heading){
        rightPodPosition = right2.getCurrentPosition() + right1.getCurrentPosition();
        leftPodPosition  = left2.getCurrentPosition()  + left1.getCurrentPosition();

        power = Math.abs(forwardDrive) + Math.abs(strafeDrive);

        aTan = Math.toDegrees(-Math.atan2(strafeDrive, -forwardDrive)) + 180;

        if (strafeDrive + forwardDrive == 0){
            aTan = 180;
        }

        currentAngle = rightPodPosition/encoderTicksPerDegree;
        newAngle = aTan;

        rotations = Math.floor(currentAngle/360);
        newAngle = newAngle + 360 * rotations;
        opposite = newAngle + 180;

        if(currentAngle < 0 && newAngle > 0){ // normal - dealer
            distance = newAngle - currentAngle;
        } else if (currentAngle > 0 && newAngle < 0){
            distance = currentAngle - newAngle;
        } else {
            distance = Math.abs(Math.abs(currentAngle) - Math.abs(newAngle));
        }

        if(currentAngle < 0 && opposite > 0) { //opo - dealer
            oppositedistance = opposite - currentAngle;
        } else if (currentAngle > 0 && opposite < 0){
            oppositedistance = currentAngle - opposite;
        } else {
            oppositedistance = Math.abs(Math.abs(currentAngle) - Math.abs(opposite));
        }

        //decide what way is shorter. for example if currentAngle is 350 and new =Angle is 370 then back to 10

        if (distance > Math.abs(Math.abs(currentAngle) - Math.abs(newAngle + 360 ))) {
            newAngle = newAngle + 360;
            if(currentAngle < 0 && newAngle > 0){ // normal - dealer
                distance = newAngle - currentAngle;
            } else if (currentAngle > 0 && newAngle < 0){
                distance = currentAngle - newAngle;
            } else {
                distance = Math.abs(Math.abs(currentAngle) - Math.abs(newAngle));
            }
        }
        else if (distance > Math.abs(Math.abs(currentAngle) - Math.abs(newAngle - 360))) {
            newAngle = newAngle - 360;
            if(currentAngle < 0 && newAngle > 0){ // normal - dealer
                distance = newAngle - currentAngle;
            } else if (currentAngle > 0 && newAngle < 0){
                distance = currentAngle - newAngle;
            } else {
                distance = Math.abs(Math.abs(currentAngle) - Math.abs(newAngle));
            }
        }
        else {
            distance = Math.abs(Math.abs(currentAngle) - Math.abs(newAngle));
        }

        if(oppositedistance > Math.abs((opposite + 360) - Math.abs(currentAngle))) {//does the same for the opposite
            opposite += 360;
            if(currentAngle < 0 && opposite > 0) { //opo - dealer
                oppositedistance = opposite - currentAngle;
            } else if (currentAngle > 0 && opposite < 0){
                oppositedistance = currentAngle - opposite;
            } else {
                oppositedistance = Math.abs(Math.abs(currentAngle) - Math.abs(opposite));
            }
        }
        else if (oppositedistance > Math.abs((opposite - 360) - Math.abs(currentAngle))){
            opposite -= 360;
            if(currentAngle < 0 && opposite > 0) { //opo - dealer
                oppositedistance = opposite - currentAngle;
            } else if (currentAngle > 0 && opposite < 0){
                oppositedistance = currentAngle - opposite;
            } else {
                oppositedistance = Math.abs(Math.abs(currentAngle) - Math.abs(opposite));
            }
        }

        if(oppositedistance < distance){
            finalAngle = opposite;
            wheelDirection = -1;
        } else {
            finalAngle = newAngle;
            wheelDirection = 1;
        }

        if((aTan < 315 && aTan > 225) || (aTan < 135 && aTan > 45)){
            turnEncoder = heading * 45 * encoderTicksPerDegree;
            turnPower = 0;
        }else{
            turnEncoder = 0;
            turnPower = heading;
        }

        turnPowerRight = odoPID(encoderTicksPerDegree *  finalAngle - turnEncoder, rightPodPosition);
        turnPowerLeft = odoPID(encoderTicksPerDegree *  finalAngle + turnEncoder, leftPodPosition);
    }

    public void resetDriveEncoders() {
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double odoPID(double target, double current){
        GeneralPIDPreviousError = GeneralPIDError;
        GeneralPIDError = target - current;
        GeneralPIDLastTime = GeneralPIDCurrentTime;
        GeneralPIDCurrentTime = (double) System.nanoTime()/1E9;
        time = GeneralPIDCurrentTime - GeneralPIDLastTime;
        GeneralPIDTotalError += time * GeneralPIDError;
        GeneralPIDTotalError = GeneralPIDTotalError < GeneralPIDMinIntegral ? GeneralPIDMinIntegral: Math.min(GeneralPIDMaxIntegral, GeneralPIDTotalError);

        GeneralPIDMotorPower = (GeneralP * GeneralPIDError)
                + (GeneralI * GeneralPIDTotalError)
                + (GeneralD * (GeneralPIDError - GeneralPIDPreviousError) / time)
                + (GeneralF * (GeneralPIDError/Math.abs(GeneralPIDError)));
        if (Double.isNaN(GeneralPIDMotorPower)){
            GeneralPIDMotorPower = 0;
        }
        return GeneralPIDMotorPower;
    }

    public void runOpMode(){}
}

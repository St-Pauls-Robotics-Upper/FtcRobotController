package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ArmSubsystem {

    //create devices
    private MotorEx toddMotor = null;
    private MotorEx bobMotor = null;
    public MotorEx jermeetMotor = null;

    private Servo sinServo = null;
    private CRServo holyServo = null;
    private RevTouchSensor ctSensor = null;

    private PIDFController bobPID  = new PIDFController(15.0, 100, 0.02, 0.1);
    private PIDFController toddPID = new PIDFController(10.0, 50, 0.01, 0.1);

    private double bobAccu = 0;

    private double toddAccu = 0;
    private double stopPosition = 0;
    private boolean isStopping = false;

    //desired positions
    public double bobDesiredPosition = 0;
    public double toddDesiredPosition = 0;
    public double sinDesiredPosition = 0;
    public double holyDesiredPosition = 0;

    ArmSubsystem(HardwareMap hardwareMap) {
        bobMotor  = new MotorEx(hardwareMap, "Bob");
        toddMotor = new MotorEx(hardwareMap, "Todd");
        jermeetMotor = new MotorEx(hardwareMap, "jermeet");

        sinServo = hardwareMap.get(Servo.class, "sins");
        holyServo = hardwareMap.get(CRServo.class, "holy");

        ctSensor = hardwareMap.get(RevTouchSensor.class, "CTS");

        sinServo.setPosition(1);

        bobAccu = 0;
        bobMotor.resetEncoder();
        bobPID.reset();

        toddAccu = 0;
        toddMotor.resetEncoder();
        toddPID.reset();
        stopPosition = 0;
        isStopping = false;
    }

    public void update(double deltaT) {

        //control for bob -----------------------------
        bobAccu = bobDesiredPosition;

        double bobAllowedError = 200;
        if (bobMotor.getCurrentPosition() > bobAccu + bobAllowedError) {
            bobAccu = bobMotor.getCurrentPosition() - bobAllowedError;
        }
        if (bobMotor.getCurrentPosition() < bobAccu - bobAllowedError) {
            bobAccu = bobMotor.getCurrentPosition() + bobAllowedError;
        }

        bobPID.setSetPoint(bobAccu);
        double bobCurrentPos = bobMotor.getCurrentPosition();
        double bobPower = bobPID.calculate(bobCurrentPos);
        bobMotor.setVelocity(bobPower);

        //control for todd -----------------------------
        boolean currentIsStopping = ctSensor.isPressed();
        if (currentIsStopping && !isStopping) {
            stopPosition = toddMotor.getCurrentPosition();
        }
        isStopping = currentIsStopping;

        toddAccu = toddDesiredPosition;

        double toddAllowedError = 150;
        if (toddMotor.getCurrentPosition() > toddAccu + toddAllowedError) {
            toddAccu = toddMotor.getCurrentPosition() - toddAllowedError;
        }
        if (toddMotor.getCurrentPosition() < toddAccu - toddAllowedError) {
            toddAccu = toddMotor.getCurrentPosition() + toddAllowedError;
        }
        if (isStopping) {
            toddAccu = Math.max(stopPosition, toddAccu);
            stopPosition = toddMotor.getCurrentPosition();
        }
        toddPID.setSetPoint(toddAccu);
        double toddCurrentPos = toddMotor.getCurrentPosition();
        double toddPower = toddPID.calculate(toddCurrentPos);
        toddMotor.setVelocity(toddPower);

        //control for sin
        sinServo.setPosition(1 - sinDesiredPosition);

        //control for holy
        holyServo.setPower(holyDesiredPosition);
    }
}

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
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ArmSubsystem {

    //Drive Motor and IMU
    private MotorEx rotatorMotor = null;
    private MotorEx slideMotor = null;

    private double rotatorDesiredPos = 0;
    private double slideDesiredPos = 0;


    /*===============================*/
    /*     Initialize Hardware       */
    /*===============================*/
    ArmSubsystem(HardwareMap hardwareMap) {
        //initialize hardware
        rotatorMotor = new MotorEx(hardwareMap, "Bob", 537.6, 340);
        slideMotor = new MotorEx(hardwareMap, "Todd", 537.6, 340);

        rotatorMotor.setRunMode(Motor.RunMode.PositionControl);
        slideMotor.setRunMode(Motor.RunMode.PositionControl);

        rotatorMotor.setVeloCoefficients(1.5, 0.002, 0.001);
        slideMotor.setVeloCoefficients(1.5, 0.002, 0.001);

        rotatorMotor.setFeedforwardCoefficients(0, 0, 0);
        slideMotor.setFeedforwardCoefficients(0, 0, 0);
    }

    public void reset() {
        rotatorMotor.resetEncoder();
        rotatorDesiredPos = 0;
        slideMotor.resetEncoder();
        slideDesiredPos = 0;
    }

    public void update(double rotator, double length) {

    }
}

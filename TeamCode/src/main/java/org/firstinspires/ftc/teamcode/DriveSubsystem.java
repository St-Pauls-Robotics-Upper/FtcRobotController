package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
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

public class DriveSubsystem {

    //Drive Motor and IMU
    private MotorEx flMotor = null;
    private MotorEx frMotor = null;
    private MotorEx blMotor = null;
    private MotorEx brMotor = null;
    private GyroEx gyroscope = null;

    // Locations of the wheels relative to the robot center.
    final private Translation2d flWheelPos = new Translation2d(0.12, 0.15);
    final private Translation2d frWheelPos = new Translation2d(0.12, -0.15);
    final private Translation2d blWheelPos = new Translation2d(-0.12, 0.15);
    final private Translation2d brWheelPos = new Translation2d(-0.12, -0.15);

    //Kinematics
    private MecanumDriveKinematics driveKinematics = new MecanumDriveKinematics(
            flWheelPos, frWheelPos,
            blWheelPos, brWheelPos
    );

    //Chassis & Wheel speed objects
    private ChassisSpeeds driveChassisSpeed = new ChassisSpeeds(0, 0, 0);
    private MecanumDriveWheelSpeeds driveWheelSpeeds = new MecanumDriveWheelSpeeds();

    //drive functions
    private void genericDrive() {
        driveWheelSpeeds = driveKinematics.toWheelSpeeds(driveChassisSpeed);

        double fl = driveWheelSpeeds.frontLeftMetersPerSecond;
        double fr = driveWheelSpeeds.frontRightMetersPerSecond;
        double bl = driveWheelSpeeds.rearLeftMetersPerSecond;
        double br = driveWheelSpeeds.rearRightMetersPerSecond;

        //wheel radius = 0.047m
        //wheel circumfrance = 0.295
        //  m/s -> rad/s = 2pi/c = 21.29
        final double velocityMultiplyer = 21.29;

        flMotor.setVelocity(fl * velocityMultiplyer, AngleUnit.RADIANS);
        frMotor.setVelocity(fr * velocityMultiplyer, AngleUnit.RADIANS);
        blMotor.setVelocity(bl * velocityMultiplyer, AngleUnit.RADIANS);
        brMotor.setVelocity(br * velocityMultiplyer, AngleUnit.RADIANS);
    }

    //Drive Functions
    public void driveUsingChassisVectors(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        driveChassisSpeed = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        genericDrive();
    }

    private Rotation2d fieldHomeRotation = new Rotation2d();
    public void resetFieldHomeRotation() {
        fieldHomeRotation = gyroscope.getRotation2d();
        lockedHeading = gyroscope.getHeading();
    }

    public void driveUsingFieldVectors(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        driveChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond,
                gyroscope.getRotation2d().rotateBy(fieldHomeRotation.unaryMinus()));
        genericDrive();
    }

    //rotation holding PID
    boolean usePidLock = false;
    PIDFController headingLockPID = new PIDFController(1,0.1,0.1,1);
    double lockedHeading = 0;

    /*===============================*/
    /*     Initialize Hardware       */
    /*===============================*/
    DriveSubsystem(HardwareMap hardwareMap) {
        //initialize hardware
        flMotor = new MotorEx(hardwareMap, "flMotor", 537.6, 340);
        frMotor = new MotorEx(hardwareMap, "frMotor", 537.6, 340);
        blMotor = new MotorEx(hardwareMap, "blMotor", 537.6, 340);
        brMotor = new MotorEx(hardwareMap, "brMotor", 537.6, 340);

        frMotor.setInverted(true);
        brMotor.setInverted(true);

        flMotor.setRunMode(Motor.RunMode.VelocityControl);
        frMotor.setRunMode(Motor.RunMode.VelocityControl);
        blMotor.setRunMode(Motor.RunMode.VelocityControl);
        brMotor.setRunMode(Motor.RunMode.VelocityControl);

        flMotor.setVeloCoefficients(0.02, 0.004, 0.06);
        frMotor.setVeloCoefficients(0.02, 0.004, 0.06);
        blMotor.setVeloCoefficients(0.02, 0.004, 0.06);
        brMotor.setVeloCoefficients(0.02, 0.004, 0.06);

        flMotor.setFeedforwardCoefficients(0, 0, 0);
        frMotor.setFeedforwardCoefficients(0, 0, 0);
        blMotor.setFeedforwardCoefficients(0, 0, 0);
        brMotor.setFeedforwardCoefficients(0, 0, 0);

        //setup imu
        RevIMU imu = new RevIMU(hardwareMap, "imu");
        imu.init();
        imu.reset();
        gyroscope = imu;

        resetFieldHomeRotation();
    }

    public void update(double vx, double vy, double vo) {
        final double manualRotationFactor = 120;
        double heading = gyroscope.getHeading();
        double pidRotationValue = headingLockPID.calculate(heading);
        double outputRotation = 0;
        if (Math.abs(vo) < 0.1 && usePidLock) {
            outputRotation = pidRotationValue;
        } else {
            outputRotation = vo * manualRotationFactor;
            lockedHeading = heading;
            headingLockPID.reset();
        }

        driveUsingFieldVectors(
                vx * 1.6,
                vy * 1.6,
                outputRotation);
    }
}

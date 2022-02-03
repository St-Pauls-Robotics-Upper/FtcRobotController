package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class DriveSubsystem {

    //Drive Motor and IMU
    public MotorEx flMotor = null;
    public MotorEx frMotor = null;
    public MotorEx blMotor = null;
    public MotorEx brMotor = null;
    public GyroEx gyroscope = null;

    //Drive pid
    private PIDFCoefficients bmf = new PIDFCoefficients(0.01, 0.005, 0, 1/3046.4);
    private PIDFController flPID = new PIDFController(bmf.p, bmf.i, bmf.d, bmf.f);
    private PIDFController frPID = new PIDFController(bmf.p, bmf.i, bmf.d, bmf.f);
    private PIDFController blPID = new PIDFController(bmf.p, bmf.i, bmf.d, bmf.f);
    private PIDFController brPID = new PIDFController(bmf.p, bmf.i, bmf.d, bmf.f);

    private double flDesirTicPos = 0;
    private double frDesirTicPos = 0;
    private double blDesirTicPos = 0;
    private double brDesirTicPos = 0;


    final double trackWidth = 22;

    private DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackWidth);

    //Odometry
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d(0,0, new Rotation2d()));

    //Chassis & Wheel speed objects
    private ChassisSpeeds targetChassisSpeed = new ChassisSpeeds(0, 0, 0);
    private DifferentialDriveWheelSpeeds targetWheelSpeeds = new DifferentialDriveWheelSpeeds();

    boolean usePID = false;
    boolean slow = false;

    final double d2m = 0.0008726;
    final double c2m = 0.00058437;

    //drive functions
    private void genericDrive(double deltaT) {
        targetWheelSpeeds = driveKinematics.toWheelSpeeds(targetChassisSpeed);

        double lft = targetWheelSpeeds.leftMetersPerSecond;
        double rit = targetWheelSpeeds.rightMetersPerSecond;

        //wheel radius = 0.047m
        //wheel circumfrance = 0.295
        //  m/s -> rad/s = 2pi/c = 21.29
        final double circumf = 0.295;
        final double velocityMultiplyer = 1711; //1760 tick / meter
        final double maxMPS = 1.78;

        if (usePID) {
            flDesirTicPos += velocityMultiplyer * lft * deltaT;
            frDesirTicPos += velocityMultiplyer * rit * deltaT;
            blDesirTicPos += velocityMultiplyer * lft * deltaT;
            brDesirTicPos += velocityMultiplyer * rit * deltaT;

            flMotor.set(flPID.calculate(flMotor.getCurrentPosition() - flDesirTicPos));
            frMotor.set(frPID.calculate(frMotor.getCurrentPosition() - frDesirTicPos));
            blMotor.set(blPID.calculate(blMotor.getCurrentPosition() - blDesirTicPos));
            brMotor.set(brPID.calculate(brMotor.getCurrentPosition() - brDesirTicPos));
        } else {
            flMotor.set(lft / maxMPS);
            frMotor.set(rit / maxMPS);
            blMotor.set(lft / maxMPS);
            brMotor.set(rit / maxMPS);
        }
    }

    //Drive Functions
    public void driveUsingChassisVectors(double deltaT, double vxMPS, double omegaRPS, boolean useSmoothing) {

        if (useSmoothing) {
            final double multiplyer = d2m;
            final double leftSpeed  = multiplyer * (flMotor.getCorrectedVelocity() + blMotor.getCorrectedVelocity()) * 0.5;
            final double rightSpeed = multiplyer * (frMotor.getCorrectedVelocity() + brMotor.getCorrectedVelocity()) * 0.5;

            final DifferentialDriveWheelSpeeds currentWheelSpeeds = new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
            final ChassisSpeeds currentChassisSpeed = driveKinematics.toChassisSpeeds(currentWheelSpeeds);

            final double currentVx = currentChassisSpeed.vxMetersPerSecond;
            final double currentVo = currentChassisSpeed.omegaRadiansPerSecond;
            final double maxVelocity = slow ? 0.3 : 1;
            final double smoothVxMPS = MathUtils.clamp(vxMPS, MathUtils.clamp(currentVx * 1 - 0.4, -1.0 * maxVelocity, -0.1), MathUtils.clamp(currentVx * 1 + 0.4, 0.1, maxVelocity));
//            final double smoothVoRPS = MathUtils.clamp(omegaRPS, MathUtils.clamp(currentVo * 1 - 0.5, -1, 0), MathUtils.clamp(currentVo * 1 + 0.5, 0, 1));

            targetChassisSpeed = new ChassisSpeeds(smoothVxMPS, 0, omegaRPS);
        } else {
            targetChassisSpeed = new ChassisSpeeds(vxMPS, 0, omegaRPS);
        }

        genericDrive(deltaT);
    }

    private Rotation2d fieldHomeRotation = new Rotation2d();
    public void resetFieldHomeRotation() {
        fieldHomeRotation = gyroscope.getRotation2d();
    }

    public void resetEncoders() {
        flMotor.resetEncoder();
        frMotor.resetEncoder();
        blMotor.resetEncoder();
        brMotor.resetEncoder();

        flPID.reset();
        frPID.reset();
        blPID.reset();
        brPID.reset();
    }

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

        flMotor.setRunMode(Motor.RunMode.RawPower);
        frMotor.setRunMode(Motor.RunMode.RawPower);
        blMotor.setRunMode(Motor.RunMode.RawPower);
        brMotor.setRunMode(Motor.RunMode.RawPower);

        //setup imu
        RevIMU imu = new RevIMU(hardwareMap, "imu");
        imu.init();
        imu.reset();
        gyroscope = imu;

        //setup odometry
        resetFieldHomeRotation();
        resetEncoders();
    }

   /* public void qwqpwp(double love, double love1, double gayscale){
        for(int i = 0; i < 100; i ++){
            int yuanda_gay_scale = 0;
            yuanda_gay_scale ++;
        }
    }*/

    public void odometryUpdate(double deltaT) {
        final double multiplyer = c2m;
        final double leftPos  = multiplyer * (flMotor.getCurrentPosition() + blMotor.getCurrentPosition()) * 0.5;
        final double rightPos = multiplyer * (frMotor.getCurrentPosition() + brMotor.getCurrentPosition()) * 0.5;

        odometry.update(gyroscope.getRotation2d(), leftPos, rightPos);
    }
}

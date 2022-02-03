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
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorsSubsystem {

    //Drive Motor and IMU
    private Rev2mDistanceSensor distanceSensor = null;
    private RevColorSensorV3 colorSensor = null;

    SensorsSubsystem(HardwareMap hardwareMap) {
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
    }

    double getSensorDistance() {
        return distanceSensor.getDistance(DistanceUnit.METER);
    }

    double getSensorColor() {
        return colorSensor.getRawLightDetected();
    }
}

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="PID Experiment", group="Linear Opmode")
public class PIDControlExperiment extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //drive motors
    private MotorEx aMotor = null;
    private PIDFController aPIDController = null;
    private double aMotorDesiredPosition = 0;

    //ran when pressed run
    @Override public void runOpMode() {
        aMotor = new MotorEx(hardwareMap, "flMotor", 537.6, 340);
        aMotor.setRunMode(MotorEx.RunMode.RawPower);
        aPIDController = new PIDFController(15.0, 0.02, 0.01, 0.1);

        waitForStart();

        double lastElapsedTime = runtime.seconds();
        while (opModeIsActive()) {
            double currentTime = runtime.seconds();
            double deltaT = currentTime - currentTime;
            lastElapsedTime = currentTime;

            final double pi = 3.1415;
            double gamepadInput = gamepad1.left_stick_y;
            double speedMPS = gamepadInput * 1.6;
            double velocity = speedMPS / 1.6;
//            aMotorDesiredPosition += velocity * deltaT;
            aMotor.set(velocity);

            telemetry.addData("Gamepad Input", gamepadInput);
            telemetry.addData("Motor Velocity", speedMPS);
            telemetry.addData("Motor True Velocity", aMotor.getVelocity() * 0.000558);
            telemetry.addData("Position", aMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
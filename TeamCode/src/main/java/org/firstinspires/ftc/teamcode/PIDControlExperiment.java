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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="PID Experiment", group="Linear Opmode")
public class PIDControlExperiment extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //drive motors
    private MotorEx aMotor = null;
    private PIDFController aPIDController = null;
    private double aMotorDesiredTickPosition = 0;

    //ran when pressed run
    @Override public void runOpMode() {
        aMotor = new MotorEx(hardwareMap, "flMotor", 537.6, 340);
        aMotor.setRunMode(MotorEx.RunMode.RawPower);
        aPIDController = new PIDFController(0.0005, 1, 0.0001, 1/3046.4); //100 divide by max tick per second

        waitForStart();

        double lastElapsedTime = runtime.seconds();
        while (opModeIsActive()) {
            double currentTime = runtime.seconds();
            double deltaT = currentTime - lastElapsedTime;
            lastElapsedTime = currentTime;



            //PID thing
            final double pi = 3.1415;
            double gamepadInput = gamepad1.left_stick_y;
            double speedRPS = gamepadInput;
            double speedTPS = speedRPS * 537.6;
            aMotorDesiredTickPosition += speedTPS * deltaT;
            double calculatedPower = aPIDController.calculate(aMotor.getCurrentPosition() - aMotorDesiredTickPosition);
            aMotor.set(calculatedPower);

            telemetry.addData("Gamepad Input", gamepadInput);
            telemetry.addData("Desired TPS", speedTPS);
            telemetry.addData("Motor True TPS", aMotor.getCorrectedVelocity());
            telemetry.addData("Position", aMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
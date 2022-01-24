package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Auto", group="Linear Opmode")
public class BasicAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //create devices
    private MotorEx flMotor = null;
    private MotorEx frMotor = null;
    private MotorEx blMotor = null;
    private MotorEx brMotor = null;

    @Override
    public void runOpMode() {
        //initialize devices
        flMotor = new MotorEx(hardwareMap, "flMotor");
        frMotor = new MotorEx(hardwareMap, "frMotor");
        blMotor = new MotorEx(hardwareMap, "blMotor");
        brMotor = new MotorEx(hardwareMap, "brMotor");

        flMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //set motor direction
        frMotor.setInverted(true);
        brMotor.setInverted(true);

        //wait to start
        waitForStart();
        runtime.reset();

//        while (opModeIsActive() && runtime.seconds() < 25.0) {};

        flMotor.set(0.7);
        frMotor.set(0.7);
        blMotor.set(0.7);
        brMotor.set(0.7);

        while (opModeIsActive() && runtime.seconds() < 1.5) {};

        flMotor.stopMotor();
        frMotor.stopMotor();
        blMotor.stopMotor();
        brMotor.stopMotor();
    }
}






















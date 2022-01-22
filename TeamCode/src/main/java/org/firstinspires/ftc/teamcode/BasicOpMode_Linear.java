package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //create devices
    private MotorEx flMotor = null;
    private MotorEx frMotor = null;
    private MotorEx blMotor = null;
    private MotorEx brMotor = null;

    PIDFController flPID  = new PIDFController(7.0, 50, 0.1, 0.1);
    PIDFController frPID  = new PIDFController(7.0, 50, 0.1, 0.1);
    PIDFController blPID  = new PIDFController(7.0, 50, 0.1, 0.1);
    PIDFController brPID  = new PIDFController(7.0, 50, 0.1, 0.1);

    private MotorEx toddMotor = null;
    private MotorEx bobMotor = null;

    private Servo sinServo = null;
    private CRServo holyServo = null;
    private CRServo susanServo = null;

    private RevTouchSensor ctSensor = null;

    @Override
    public void runOpMode() {
        //initialize devices
        flMotor = new MotorEx(hardwareMap, "flMotor");
        frMotor = new MotorEx(hardwareMap, "frMotor");
        blMotor = new MotorEx(hardwareMap, "blMotor");
        brMotor = new MotorEx(hardwareMap, "brMotor");

        bobMotor  = new MotorEx(hardwareMap, "Bob");
        toddMotor = new MotorEx(hardwareMap, "Todd");

        sinServo = hardwareMap.get(Servo.class, "sins");
        holyServo = hardwareMap.get(CRServo.class, "holy");

        susanServo = hardwareMap.get(CRServo.class, "susan");

        ctSensor = hardwareMap.get(RevTouchSensor.class, "CTS");

        PIDFController bobPID  = new PIDFController(15.0, 100, 0.02, 0.1);
        PIDFController toddPID = new PIDFController(10.0, 50, 0.01, 0.1);

        //set motor direction
        frMotor.setInverted(true);
        brMotor.setInverted(true);

        //drives
        boolean lastIsSlowMode = false;
        flPID.reset();
        frPID.reset();
        blPID.reset();
        brPID.reset();

        //set servo default position
        sinServo.setPosition(1);

        //wait to start
        waitForStart();
        runtime.reset();

        //setup for bob
        double bobAccu = 0;
        bobMotor.resetEncoder();
        bobPID.reset();

        //setup for todd
        double toddAccu = 0;
        toddMotor.resetEncoder();
        toddPID.reset();
        double stopPosition = 0;
        boolean isStopping = false;

        //main runloop
        double lastTimeMeasure = runtime.seconds();

        while (opModeIsActive()) {
            double currentTime = runtime.seconds();
            double deltaT = currentTime - lastTimeMeasure;
            lastTimeMeasure = currentTime;

            boolean currentIsStopping = ctSensor.isPressed();
            if (currentIsStopping && !isStopping) {
                stopPosition = toddMotor.getCurrentPosition();
            }
            isStopping = currentIsStopping;

            //get roughly how far todd have extended
            double roughExtensionFactor = toddMotor.getCurrentPosition() * 0.00025;

            //control for bob -----------------------------
//            double bobDt = bobPID.getPeriod();
            double bobSpeed = 800 * (1 - roughExtensionFactor * 0.5);
            bobAccu += gamepad2.left_stick_y * 600 * deltaT;
//            bobAccu = Math.min(0, bobAccu);

            telemetry.addData("Bob Delta", bobMotor.getCurrentPosition() - bobAccu);
            telemetry.addData("Bob Position", bobMotor.getCurrentPosition());

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
//            double toddDt = toddPID.getPeriod();

            toddAccu += -gamepad2.right_stick_y * 1000 * deltaT;

            telemetry.addData("Todd Delta", toddMotor.getCurrentPosition() - toddAccu);
            telemetry.addData("Todd Position", toddMotor.getCurrentPosition());

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

            //control for sins
            double amountOfSin = 1 - gamepad2.right_trigger * 0.5;
            sinServo.setPosition(amountOfSin);

            //control for holy
            holyServo.setPower(gamepad2.left_bumper ? 1 : -gamepad2.left_trigger);

            //susan
            susanServo.setPower(gamepad2.dpad_left ? -1 : gamepad2.dpad_right ? 1 : 0);

            //obtain driver parameter
            boolean isInSlowMo = gamepad1.left_bumper || gamepad1.right_bumper;
            if (!lastIsSlowMode && isInSlowMo) { //entering slow mode
                flPID.setSetPoint(flMotor.getCurrentPosition());
                frPID.setSetPoint(frMotor.getCurrentPosition());
                blPID.setSetPoint(blMotor.getCurrentPosition());
                brPID.setSetPoint(brMotor.getCurrentPosition());

                flPID.reset();
                frPID.reset();
                blPID.reset();
                brPID.reset();
            }

            lastIsSlowMode = isInSlowMo;

            double driver2turn = Math.abs(gamepad2.right_stick_x) > 0.2 ? gamepad2.right_stick_x : 0;
            double drive = -gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x + 0.3 * gamepad2.right_stick_x;
            double stray = gamepad1.left_stick_x;

            //compute motor power
            double flPower = drive + turn + stray;
            double frPower = drive - turn - stray;
            double blPower = drive + turn - stray;
            double brPower = drive - turn + stray;

            //set motor power
            if (isInSlowMo && lastIsSlowMode) {
                double speed = 600 * deltaT;
                flPID.setSetPoint(flPID.getSetPoint() + speed * flPower);
                frPID.setSetPoint(frPID.getSetPoint() + speed * frPower);
                blPID.setSetPoint(blPID.getSetPoint() + speed * blPower);
                brPID.setSetPoint(brPID.getSetPoint() + speed * brPower);

                flMotor.setVelocity(flPID.calculate(flMotor.getCurrentPosition()));
                frMotor.setVelocity(frPID.calculate(frMotor.getCurrentPosition()));
                blMotor.setVelocity(blPID.calculate(blMotor.getCurrentPosition()));
                brMotor.setVelocity(brPID.calculate(brMotor.getCurrentPosition()));
            } else {
                flMotor.set(flPower);
                frMotor.set(frPower);
                blMotor.set(blPower);
                brMotor.set(brPower);
            }

            telemetry.update();
        }
    }
}






















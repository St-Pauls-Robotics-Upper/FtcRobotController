/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private DcMotor flMotor = null;
    private DcMotor frMotor = null;
    private DcMotor blMotor = null;
    private DcMotor brMotor = null;

    private MotorEx toddMotor = null;
    private MotorEx bobMotor = null;

    private Servo sinServo = null;

    private RevTouchSensor ctSensor = null;

    @Override
    public void runOpMode() {
        //initialize devices
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");

        bobMotor  = new MotorEx(hardwareMap, "Bob");
        toddMotor = new MotorEx(hardwareMap, "Todd");

        sinServo = hardwareMap.get(Servo.class, "sins");

        ctSensor = hardwareMap.get(RevTouchSensor.class, "CTS");


        PIDFController bobPID  = new PIDFController(15.0, 0.02, 0.01, 0.1);
        PIDFController toddPID = new PIDFController(15.0, 0.02, 0.005, 0.1);

        //set motor direction
        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.REVERSE);

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
        while (opModeIsActive()) {

            boolean currentIsStopping = ctSensor.isPressed();
            if (currentIsStopping && !isStopping) {
                stopPosition = toddMotor.getCurrentPosition();
            }
            isStopping = currentIsStopping;

            //control for bob
//            double bobDt = bobPID.getPeriod();
            bobAccu += -gamepad2.left_stick_y * 15;
            bobAccu = Math.min(0, bobAccu);

            bobPID.setSetPoint(bobAccu);

            double bobCurrentPos = bobMotor.getCurrentPosition();
            double bobPower = bobPID.calculate(bobCurrentPos);
            bobMotor.setVelocity(bobPower);

            //control for todd
//            double toddDt = toddPID.getPeriod();
            toddAccu += gamepad2.right_stick_y * 25;
            if (isStopping) {
                toddAccu = Math.max(stopPosition, toddAccu);
            }

            toddPID.setSetPoint(toddAccu);

            double toddCurrentPos = toddMotor.getCurrentPosition();
            double toddPower = toddPID.calculate(toddCurrentPos);
            toddMotor.setVelocity(toddPower);

            //control for sins
            double amountOfSin = gamepad2.right_trigger;
            sinServo.setPosition(amountOfSin);

            //obtain driver parameter
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double hdrive = gamepad1.left_stick_x;

            //compute motor power
            double flPower = drive + turn - hdrive;
            double frPower = drive - turn + hdrive;
            double blPower = drive + turn + hdrive;
            double brPower = drive - turn - hdrive;

            //set motor power
            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            blMotor.setPower(blPower);
            brMotor.setPower(brPower);
        }
    }
}






















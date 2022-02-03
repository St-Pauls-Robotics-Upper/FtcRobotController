package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class ExperimentalDrive extends LinearOpMode {

    final private ElapsedTime runtime = new ElapsedTime();
    double executeStartTime = 0;

    //Subsystems
    public DriveSubsystem driveSubsystem = null;
    public ArmSubsystem armSubsystem = null;
    public SensorsSubsystem sensorsSubsystem = null;

    //Exceptions
    interface RobotException {
        boolean shouldTerminate(double time);
    }

    //Rotate Task
    final private PIDFController rotateOnlyPIDF = new PIDFController(0.06, 3, 0.02, 1);
    private Pose2d desiredPointingPosition = new Pose2d();
    private double desiredPointing = 0;
    private boolean usePosition = false;
    private boolean waitForRotation = false;
    private Accuracy rotationMode = Accuracy.High;

    //Drive Task
    final private PIDFController drivePID = new PIDFController(2, 50, 0.7, 1);
    final private PIDFController rotatePIDF = new PIDFController(0.06, 4, 0.05, 1);

    private Pose2d desiredPosition = new Pose2d();
    private Accuracy positionMode = Accuracy.High;
    private boolean waitForPosition = false;

    //Timer Task
    private double desiredTime = 0;
    private boolean waitForTimer = false;

    //Constants
    final private double nearZero = 0.0000001;
    final private double nearPi   = 3.1415926;

    /*---------------------------------*/
    /*            Exceptions           */
    /*---------------------------------*/
    ArrayList<RobotException> exceptionsList = new ArrayList<RobotException>();
    final void registerException(RobotException exception) {
        exceptionsList.add(exception);
    }
    boolean exceptionTriggered = false;

    enum Accuracy {
        High, Low, Rough
    }

    /*---------------------------------*/
    /*              Tasks              */
    /*---------------------------------*/

    final void taskTimer(double time) {
        desiredTime = time + runtime.seconds();
        waitForTimer = true;
    }

    final void taskGoToPosition(double x, double y, Accuracy accuracy) {
        drivePID.reset();
        rotatePIDF.reset();

        this.desiredPosition = new Pose2d(x, y, new Rotation2d());
        this.positionMode = accuracy;
        waitForPosition = true;
    }

    final void taskGoToPositionRelative(double x, double y, Accuracy accuracy) {
        double distance = Math.sqrt(x * x + y * y);

        Pose2d position = driveSubsystem.odometry.getPoseMeters();

        double angle = 0;
        if (distance > nearZero && Math.abs(x) > nearZero) angle = Math.atan(y / x) + position.getHeading();
        while (angle >  nearPi) angle -= nearPi * 2;
        while (angle < -nearPi) angle += nearPi * 2;

        double dy = Math.sin(angle) * distance;
        double dx = Math.cos(angle) * distance;

        taskGoToPosition(position.getX() + dx, position.getY() + dy, accuracy);
    }

    final void taskFacePosition(double x, double y, Accuracy accuracy) {
        desiredPointingPosition = new Pose2d(x, y, new Rotation2d());
        usePosition = true;
        rotationMode = accuracy;
        waitForRotation = true;

        rotateOnlyPIDF.reset();
    }

    final void taskFaceRotation(double o, Accuracy accuracy) {
        desiredPointing = o;
        usePosition = false;
        rotationMode = accuracy;
        waitForRotation = true;

        rotateOnlyPIDF.reset();
    }

    /*---------------------------------*/
    /*              State              */
    /*---------------------------------*/
    private double stateVx = 0;
    private double stateVo = 0;
    final void setDriveState(double vx, double vo) {
        this.stateVx = vx;
        this.stateVo = vo;
    }

    final void resetDriveState() {
        stateVx = 0;
        stateVo = 0;
    }

    /*---------------------------------*/
    /*    Periodic Update Function     */
    /*---------------------------------*/
    private double lastElapsedTime = 0;
    private Pose2d lastPosition = new Pose2d();
    final boolean update() {
        //calculate deltaT
        double currentTime = runtime.seconds();
        double deltaT = currentTime - lastElapsedTime;
        lastElapsedTime = currentTime;

        //update arm subsystem
        armSubsystem.update(deltaT);

        //Drive Loop
        driveSubsystem.odometryUpdate(deltaT);

        Pose2d position = driveSubsystem.odometry.getPoseMeters();
        double measuredVx = (position.getX() - lastPosition.getX()) * deltaT;
        double measuredVy = (position.getY() - lastPosition.getY()) * deltaT;
        double measuredChassisSpeed = Math.sqrt(measuredVx * measuredVx + measuredVy * measuredVy);
        double measuredChassisSpeedRot = position.getHeading() - lastPosition.getHeading();
        while (measuredChassisSpeedRot >  nearPi) measuredChassisSpeedRot -= nearPi * 2;
        while (measuredChassisSpeedRot < -nearPi) measuredChassisSpeedRot += nearPi * 2;
        measuredChassisSpeedRot /= deltaT;

        lastPosition = position;

        double targetVx = stateVx;
        double targetVo = stateVo;

        /*-----------------------------*/
        /*            Tasks            */
        /*-----------------------------*/

        //Timer Implementation
        if (runtime.seconds() >= desiredTime && waitForTimer) {
            waitForTimer = false;
        }

        //Position Implementation
        if (waitForPosition) {
            //calculate rotation needed to face the desired location
            final double dx = desiredPosition.getX() - position.getX();
            final double dy = desiredPosition.getY() - position.getY();

            final double distance = Math.sqrt(dx * dx + dy * dy);

            double dAngle = 0;
            if (distance > nearZero && Math.abs(dx) > nearZero) dAngle = Math.atan2(dy, dx) - position.getHeading();
            while (dAngle >  nearPi) dAngle -= nearPi * 2;
            while (dAngle < -nearPi) dAngle += nearPi * 2;

            double collapsedDAngle = dAngle;
            while (collapsedDAngle >  nearPi/2) collapsedDAngle -= nearPi;
            while (collapsedDAngle < -nearPi/2) collapsedDAngle += nearPi;

            double dPosition = 0;
            if (distance > nearZero) dPosition = Math.pow(Math.cos(dAngle), 3) * distance;

            double rotationCarelessDistance = 0.05;
            switch (positionMode) {
                case High:
                    rotationCarelessDistance = 0.1;
                case Low:
                    rotationCarelessDistance = 0.2;
                case Rough:
                    rotationCarelessDistance = 0.3;
            }

            double drive = -drivePID.calculate(dPosition);
            double rotate = -rotatePIDF.calculate(collapsedDAngle);
            if (distance < rotationCarelessDistance) rotate = 0;

            telemetry.addLine("Task: Approaching Position");
            telemetry.addData("dPosition", dPosition);
            telemetry.addData("dAngle", dAngle);

            switch (positionMode) {
                case Rough:
                    if (Math.abs(distance) < 0.3 && Math.abs(dPosition) < 0.1) {
                        waitForPosition = false;
                    }
                    break;
                case Low:
                    if (Math.abs(distance) < 0.2 && Math.abs(dPosition) < 0.05 && Math.abs(drive) < 0.2 && Math.abs(rotate) < 0.2) {
                        waitForPosition = false;
                    }
                    break;
                case High:
                    if (Math.abs(distance) < 0.05 && Math.abs(dPosition) < 0.01 && Math.abs(drive) < 0.1 && measuredChassisSpeed < 0.2 && Math.abs(rotate) < 0.1) {
                        waitForPosition = false;
                    }
            }

            targetVx += drive;
            targetVo += rotate;
        }

        //Rotation Implementation
        if (waitForRotation && !waitForPosition) {
            //calculate rotation needed to face the desired location
            double dAngle = 0;
            if (usePosition) {
                final double dx = desiredPointingPosition.getX() - position.getX();
                final double dy = desiredPointingPosition.getY() - position.getY();
                final double distance2 = dx * dx + dy * dy;

                if (distance2 > nearZero && Math.abs(dx) > nearZero) dAngle = Math.atan2(dy, dx) - position.getHeading();
            } else {
                dAngle = desiredPointing - position.getHeading();
            }

            while (dAngle >  nearPi) dAngle -= nearPi * 2;
            while (dAngle < -nearPi) dAngle += nearPi * 2;

            double rotate = -rotateOnlyPIDF.calculate(dAngle);

            if (Math.abs(rotate) < 0.1) {
                switch (rotationMode) {
                    case Rough:
                        if (Math.abs(dAngle) < 0.2) { //0.2 rad ≈ 11.5 deg
                            waitForRotation = false;
                        }
                        break;
                    case Low:
                        if (Math.abs(dAngle) < 0.1 && measuredChassisSpeedRot < 0.1) { //0.1 rad ≈ 5.7 deg
                            waitForRotation = false;
                        }
                        break;
                    case High:
                        if (Math.abs(dAngle) < 0.05 && measuredChassisSpeedRot < 0.02) { //0.05 rad ≈ 3 deg
                            waitForRotation = false;
                        }
                }
            }

            targetVo += rotate;
        }

        driveSubsystem.driveUsingChassisVectors(deltaT, targetVx, targetVo, true);

        telemetry.addData("x", position.getX());
        telemetry.addData("y", position.getY());
        telemetry.addData("o", position.getHeading());
        telemetry.update();

        for(RobotException exception: exceptionsList) {
            if (exception.shouldTerminate(runtime.seconds() - executeStartTime)) {
                return true;
            }
        }

        boolean tasksNotComplete = waitForTimer || waitForPosition || waitForRotation;

        //return true to escape loop
        if (!opModeIsActive()) { return true; }

        return !tasksNotComplete;
    }

    final void resetTasks() {
        waitForTimer = false;
        waitForPosition = false;
        waitForRotation = false;
        exceptionsList = new ArrayList<>();
    }

    final void execute() {
        executeStartTime = runtime.seconds();

        while (true) {
            boolean breaking = update();
            if (breaking) { break; }
        }

        resetTasks();
        resetDriveState();
    }

    //ran when pressed run
    @Override final public void runOpMode() {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        armSubsystem = new ArmSubsystem(hardwareMap);
        sensorsSubsystem = new SensorsSubsystem(hardwareMap);

        waitForStart();

        autonomous();

        double lastElapsedTime = 0;
        while (opModeIsActive() && !gamepad1.left_bumper) {
            //calculate deltaT
            double currentTime = runtime.seconds();
            double deltaT = currentTime - lastElapsedTime;
            lastElapsedTime = currentTime;

            armSubsystem.update(deltaT);

            driveSubsystem.odometryUpdate(deltaT);

            double drive = -gamepad1.left_stick_y * 1.78;
            double turn  = -gamepad1.left_stick_x * 0.5;
            driveSubsystem.driveUsingChassisVectors(deltaT, drive, turn, true);

            Pose2d currentPosition = driveSubsystem.odometry.getPoseMeters();

            telemetry.addData("Color Sensor Reading", sensorsSubsystem.getSensorColor());
            telemetry.addData("Distance Sensor Reading", sensorsSubsystem.getSensorDistance());
            telemetry.addData("Pos", currentPosition.toString());
            telemetry.update();
        }

//        autonomous();
    }

    public void autonomous() {
        //inherited by children class
    }
}






















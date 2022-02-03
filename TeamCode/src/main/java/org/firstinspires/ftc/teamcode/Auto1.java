package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Auto 1", group="Linear Opmode")
public class Auto1 extends ExperimentalDrive {
    @Override
    public void autonomous() {

        armSubsystem.bobDesiredPosition = -140;
        armSubsystem.sinDesiredPosition = 0.2;

        taskGoToPosition(0.56, 0, Accuracy.High);
        registerException(time1 -> { return time1 > 1.5;});
        execute();

        double cubeDist = sensorsSubsystem.getSensorDistance();
        int position = 0;//1 or 2 or 3. 1 = low, 3 = high.
        if (cubeDist < 0.25) {//first cube
            position = 3;
        } else if (cubeDist < 0.45) {//second cube
            position = 2;
        } else {//third cube
            position = 1;
        }

        switch (position) {
            case 1:
                armSubsystem.toddDesiredPosition = 900;
                break;
            case 2:
                armSubsystem.toddDesiredPosition = 900;
                break;
            case 3:
                armSubsystem.toddDesiredPosition = 2400;
                break;
        }

        taskTimer(1);
        taskGoToPosition(0.30, 0, Accuracy.Low);
        registerException((double time) -> { return time > 1.0;});
        execute();

        switch (position) {
            case 1:
                armSubsystem.bobDesiredPosition = 160;
                armSubsystem.sinDesiredPosition = 0.4;
                break;
            case 2:
                armSubsystem.bobDesiredPosition = -200;//
                armSubsystem.sinDesiredPosition = 0.3;
                break;
            case 3:
                armSubsystem.bobDesiredPosition = -500;
                armSubsystem.sinDesiredPosition = 0.2;
                break;
        }

        taskTimer(1);
        execute();

        if (position == 1) {
            taskGoToPosition(0.40, 0, Accuracy.High);
        } else {
            taskGoToPosition(0.45, 0, Accuracy.High);
        }
        registerException(time1 -> { return time1 > 1.3;});
        execute();

        armSubsystem.holyDesiredPosition = 1;
        taskTimer(0.5);
        execute();

        armSubsystem.holyDesiredPosition = 0;

        taskGoToPosition(0.38, 0, Accuracy.Rough);
        registerException(time1 -> { return time1 > 2;});
        execute();

        armSubsystem.bobDesiredPosition = -500;
        armSubsystem.toddDesiredPosition = 700;

        taskTimer(0.5);
        execute();

        armSubsystem.toddDesiredPosition = 0;
        armSubsystem.sinDesiredPosition = 0.3;

        taskFacePosition(0.5, 0.4, Accuracy.High);
        registerException(time1 -> { return time1 > 2;});
        execute();

        taskGoToPosition(0.5, 0.4, Accuracy.High);
        registerException(time1 -> { return time1 > 5; });
        execute();

        taskFaceRotation(1.57, Accuracy.High);
        registerException(time1 -> { return time1 > 1; });
        execute();

        taskTimer(4);
        setDriveState(0.3, 0);
        registerException(time1 -> { return sensorsSubsystem.getSensorColor() > 240; });
        execute();

        taskGoToPositionRelative(0.48, 0, Accuracy.High);
        registerException(time1 -> { return time1 > 0.8; });
        execute();

        taskFaceRotation(0, Accuracy.Low);
        execute();

        taskTimer(1);
        setDriveState(-0.4, 0);
        execute();

        taskTimer(3);
        setDriveState(-0.12, 0);
        armSubsystem.jermeetMotor.set(0.8);
        execute();

        armSubsystem.jermeetMotor.set(0);

        registerException(time1 -> { return time1 > 5; });
        execute();

        taskGoToPositionRelative(0.1, 0, Accuracy.Low);
        execute();

        taskFaceRotation(0.1, Accuracy.High);
        registerException(time1 -> { return time1 > 1; });
        execute();

        taskGoToPositionRelative(0.40, 0, Accuracy.Low);
        registerException(time1 -> { return time1 > 1; });
        execute();

        taskTimer(2);
        setDriveState(0.3, 0);
        registerException(time1 -> { return sensorsSubsystem.getSensorColor() > 270; });
        execute();

        taskTimer(2);
        setDriveState(-0.3, 0);
        registerException(time1 -> { return sensorsSubsystem.getSensorColor() < 240; });
        execute();
    }
}

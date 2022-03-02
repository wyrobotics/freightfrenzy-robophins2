package org.firstinspires.ftc.teamcode.Components;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Hardware.Extender;
import org.firstinspires.ftc.teamcode.Components.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Components.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.Components.Hardware.Spinner;
import org.firstinspires.ftc.teamcode.Components.Hardware.Lighting;
import org.firstinspires.ftc.teamcode.Components.Software.OpenCVSampler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Components.Software.RealsenseLocalizer;

import java.util.concurrent.ExecutorService;

@Config
public class MainRobot extends SampleMecanumDrive {

    public Extender extender;
    public Intake intake;
    public Spinner spinner;
    public Shooter shooter;

    public Lighting lighting;

    public OpenCVSampler openCVSampler;

    public Intake.Color cubeColor = new Intake.Color(150,90,50);
    public Intake.Color sphereColor = new Intake.Color(150,150,150);

    public volatile boolean leftFlushing = false;
    public volatile boolean rightFlushing = false;

    public MainRobot(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap);
        //setLocalizer(new RealsenseLocalizer(hardwareMap, telemetry));

        extender = new Extender(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        spinner = new Spinner(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);

        lighting = new Lighting(hardwareMap, telemetry);

        openCVSampler = new OpenCVSampler(hardwareMap, telemetry);

    }

    public static double flushPower = 1;
    public static double flushTime = 250;
    public static double sweepTime = 1000;

    public void flushLeftIntake() {

        if(!(intake.getLeftColor().threshold(cubeColor) || intake.getLeftColor().threshold(sphereColor))) return;
        if(leftFlushing) {
            lighting.signalNoFreight();
            return;
        }

        ExecutorService flushExecutor = ThreadPool.newSingleThreadExecutor("Left Flush");
        flushExecutor.execute(new Runnable() {
            @Override
            public void run() {
                leftFlushing = true;
                lighting.signalFreightReceived();
                for(int i = 0; i < 1; i++) {
                    extender.openReleaser();
                    intake.setLeftPower(flushPower);
                    pause((long) flushTime);
                    if (Thread.currentThread().isInterrupted()) break;
                    intake.leftSlapIn();
                    intake.rightSlapIn();
                    pause((long) sweepTime);
                    //if(Thread.currentThread().isInterrupted()) return;
                }
                intake.setLeftPower(0);
                lighting.signalNoFreight();
                leftFlushing = false;
            }
        });

    }

    public void flushRightIntake() {

        if(!(intake.getRightColor().threshold(cubeColor) || intake.getRightColor().threshold(sphereColor))) return;
        if(rightFlushing) {
            lighting.signalNoFreight();
            return;
        }

        ExecutorService flushExecutor = ThreadPool.newSingleThreadExecutor("Right Flush");
        flushExecutor.execute(new Runnable() {
            @Override
            public void run() {
                rightFlushing = true;
                lighting.signalFreightReceived();
                for(int i = 0; i < 1; i++) {
                    extender.openReleaser();
                    intake.setRightPower(-flushPower);
                    pause((long) flushTime);
                    if (Thread.currentThread().isInterrupted()) break;
                    intake.leftSlapIn();
                    intake.rightSlapIn();
                    pause((long) sweepTime);
                    //if(Thread.currentThread().isInterrupted()) return;
                }
                intake.setRightPower(0);
                lighting.signalNoFreight();
                rightFlushing = false;
            }
        });

    }

    public void pause(long time) {
        long initTime = System.currentTimeMillis();
        while(initTime + time > System.currentTimeMillis() && !Thread.currentThread().isInterrupted()) { }
    }

}

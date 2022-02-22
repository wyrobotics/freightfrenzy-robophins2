package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

@Config
@Autonomous
public class PreloadParkRed extends LinearOpMode {

    MainRobot mainRobot;

    public static double firstExtendPower = 0.8;
    public static long firstExtend = 700;
    public static double rotatorPos = 0.4;
    public static double dPos = -1.0 / 150.0;
    public static long secondExtend = 1000;


    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();

        mainRobot.extender.closeReleaser();
        mainRobot.pause(200);

        mainRobot.extender.rotator.setPosition(0.4);
        mainRobot.pause(1000);
        mainRobot.extender.rotator.setPosition(0.9);
        mainRobot.pause(1000);
        mainRobot.extender.rotator.setPosition(0.4);
        mainRobot.pause(1000);
        mainRobot.extender.rotator.setPosition(0.9);
        mainRobot.pause(1000);
        mainRobot.extender.setExtenderPower(firstExtendPower);
        mainRobot.pause(firstExtend);
        mainRobot.extender.setExtenderPower(0);
        mainRobot.pause(100);
        mainRobot.extender.rotator.setPosition(0.4);
        mainRobot.pause(1000);
        mainRobot.extender.rotator.setPosition(0.9);
        mainRobot.pause(1000);
        while(mainRobot.extender.rotatorPosition > rotatorPos) {
            mainRobot.extender.changeRotatorPosition(dPos);
        }
        mainRobot.pause(1000);
        mainRobot.extender.setExtenderPower(firstExtendPower);
        mainRobot.pause(secondExtend);
        mainRobot.extender.setExtenderPower(0);
        mainRobot.pause(500);
        mainRobot.extender.openReleaser();
        mainRobot.pause(1000);
        mainRobot.extender.setExtenderPower(-firstExtendPower);
        mainRobot.pause(1200);
        mainRobot.extender.setExtenderPower(0);
        mainRobot.pause(400);
        mainRobot.extender.closeReleaser();
        mainRobot.pause(1000);
        mainRobot.setWeightedDrivePower(new Pose2d(-0.5,0,0));
        mainRobot.pause(2000);
        mainRobot.setWeightedDrivePower(new Pose2d());
        mainRobot.pause(200);


    }

}

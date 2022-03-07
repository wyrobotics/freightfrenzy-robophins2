package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Legacy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

@Autonomous
public class SimplyParkRed extends LinearOpMode {

    MainRobot mainRobot;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();

        mainRobot.setWeightedDrivePower(new Pose2d(-0.5,0.0,0.0));
        mainRobot.pause(1000);
        mainRobot.setWeightedDrivePower(new Pose2d());


    }

}

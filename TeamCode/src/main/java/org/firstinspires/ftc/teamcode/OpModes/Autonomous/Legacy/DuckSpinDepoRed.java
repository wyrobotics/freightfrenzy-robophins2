package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Legacy;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

public class DuckSpinDepoRed extends LinearOpMode {

    MainRobot mainRobot;

    public static double forwardDistance = 6;
    public static double strafeToDisk = 12;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        Trajectory toCarousel1 = mainRobot.trajectoryBuilder(mainRobot.getPoseEstimate())
                .forward(forwardDistance)
                .build();

        Trajectory toCarousel2 = mainRobot.trajectoryBuilder(toCarousel1.end())
                .strafeRight(strafeToDisk)
                .build();

        waitForStart();

        mainRobot.followTrajectory(toCarousel1);
        mainRobot.pause(1000);
        mainRobot.followTrajectory(toCarousel2);
        mainRobot.pause(1000);
        mainRobot.spinner.spin(0.8);
        mainRobot.pause(3000);
        mainRobot.spinner.spin(0);
        mainRobot.pause(200);

    }

}

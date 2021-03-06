package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.MainRobot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class RedDuckPREMIUM extends LinearOpMode {

    MainRobot mainRobot;

    public static double firstBackwardDistance = 24;
    public static long firstExtend = 400;
    public static double firstExtendPower = 0.8;
    public static double raiseInc = 0.45 / 3;
    public static long raiseTime = 1000;
    public static double[] raise = new double[] {0.0,0.5,0.62,0.72};
    //public static long secondExtend = 200;
    public static long[] secondExtend = new long[] {0,450,400,470};
    public static double secondExtendPower = 0.8;
    public static long releaseTime = 1000;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        Pose2d startPose = mainRobot.getPoseEstimate();

        Trajectory firstStrafe = mainRobot.trajectoryBuilder(startPose)
                .strafeRight(4)
                .build();

        Trajectory secondBackward = mainRobot.trajectoryBuilder(firstStrafe.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .back(18, SampleMecanumDrive.getVelocityConstraint(10,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory thirdForward = mainRobot.trajectoryBuilder(secondBackward.end())
                .forward(18, SampleMecanumDrive.getVelocityConstraint(10,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory fourthStrafe = mainRobot.trajectoryBuilder(thirdForward.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .strafeLeft(5)
                .build();

        Trajectory fifthBackward = mainRobot.trajectoryBuilder(fourthStrafe.end())
                .back(60)
                .build();


        mainRobot.openCVSampler.initOpenCVCamera();

        waitForStart();

        double markerPos = mainRobot.openCVSampler.getPinkX();
        int level;
        if(markerPos < 50) level = 1;
        else if(markerPos < 140) level = 3;
        else level = 2;


        telemetry.addData("Level: ", level);
        telemetry.addData("Pink: ", mainRobot.openCVSampler.getPinkX());
        telemetry.update();

        mainRobot.openCVSampler.closeOpenCVCamera();

        mainRobot.extender.closeReleaser();
        mainRobot.pause(300);

        mainRobot.followTrajectory(firstStrafe);
        mainRobot.pause(500);

        mainRobot.turn(Math.toRadians(180));

        mainRobot.followTrajectory(secondBackward);
        mainRobot.pause(500);

        mainRobot.spinner.spin(1);
        mainRobot.pause(4000);
        mainRobot.spinner.spin(0);
        mainRobot.pause(500);

        mainRobot.followTrajectory(thirdForward);
        mainRobot.pause(500);

        mainRobot.turn(Math.toRadians(180));
        mainRobot.pause(500);

        mainRobot.followTrajectory(fourthStrafe);
        mainRobot.pause(500);

        mainRobot.extender.setExtenderPower(firstExtendPower);
        mainRobot.pause(firstExtend);
        mainRobot.extender.setExtenderPower(0.0);

        mainRobot.extender.rotator.setPosition(raise[level]);
        mainRobot.pause(raiseTime);

        mainRobot.extender.setExtenderPower(secondExtendPower);
        mainRobot.pause(secondExtend[level]);
        mainRobot.extender.setExtenderPower(0.0);
        mainRobot.pause(500);

        mainRobot.extender.openReleaser();
        mainRobot.pause(1000);

        mainRobot.extender.closeReleaser();
        mainRobot.pause(200);
        mainRobot.extender.setExtenderPower(-0.8);
        mainRobot.pause(400);
        while(mainRobot.extender.inSwitch.getState()) {
            mainRobot.extender.rotator.setPosition(0.32);
        }
        mainRobot.extender.setExtenderPower(0);

        mainRobot.pause(1000);

        mainRobot.followTrajectory(fifthBackward);

        mainRobot.pause(1000);

    }

}

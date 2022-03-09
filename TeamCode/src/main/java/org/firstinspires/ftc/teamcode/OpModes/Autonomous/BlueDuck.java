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
public class BlueDuck extends LinearOpMode {

    MainRobot mainRobot;

    public static double firstBackwardDistance = 24;
    public static long firstExtend = 400;
    public static double firstExtendPower = 0.8;
    public static double raiseInc = 0.45 / 3;
    public static long raiseTime = 1000;
    public static double[] raise = new double[] {0.0,0.45,0.57,0.72};
    //public static long secondExtend = 200;
    public static long[] secondExtend = new long[] {0,500,400,470};
    public static double secondExtendPower = 0.8;
    public static long releaseTime = 1000;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        Pose2d startPose = mainRobot.getPoseEstimate();

        Trajectory firstBack = mainRobot.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-18,-3,-0.5),0,
                        SampleMecanumDrive.getVelocityConstraint(10,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory secondForward = mainRobot.trajectoryBuilder(firstBack.end())
                .splineToLinearHeading(new Pose2d(firstBack.end().getX()+44,firstBack.end().getY()+3,0),0,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory thirdForward = mainRobot.trajectoryBuilder(secondForward.end())
                .forward(60)
                .build();

        Trajectory adjust = mainRobot.trajectoryBuilder(thirdForward.end())
                .strafeRight(26)
                .build();

        mainRobot.openCVSampler.initOpenCVCamera();

        waitForStart();

        double markerPos = mainRobot.openCVSampler.getPinkX();
        int level;
        if(markerPos < 50) level = 1;
        else if(markerPos < 110) level = 3;
        else level = 2;


        telemetry.addData("Level: ", level);
        telemetry.addData("Pink: ", mainRobot.openCVSampler.getPinkX());
        telemetry.update();

        mainRobot.extender.closeReleaser();
        mainRobot.pause(300);

        mainRobot.followTrajectory(firstBack);
        mainRobot.pause(500);

        mainRobot.spinner.spin(1);
        mainRobot.pause(4000);
        mainRobot.spinner.spin(0);
        mainRobot.pause(500);

        mainRobot.followTrajectory(secondForward);
        mainRobot.pause(1000);

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

        mainRobot.followTrajectory(thirdForward);

        mainRobot.pause(1000);

    }

}

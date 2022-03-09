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
public class BlueCycle extends LinearOpMode {

    MainRobot mainRobot;

    public static double firstBackwardDistance = 24;
    public static long firstExtend = 400;
    public static double firstExtendPower = 0.8;
    public static double raiseInc = 0.45 / 3;
    public static long raiseTime = 500;
    public static double[] raise = new double[] {0.0,0.5,0.57,0.72};
    //public static long secondExtend = 200;
    public static long[] secondExtend = new long[] {0,500,400,470};
    public static double secondExtendPower = 0.8;
    public static long releaseTime = 1000;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        Pose2d startPose = mainRobot.getPoseEstimate();

        Trajectory firstBack = mainRobot.trajectoryBuilder(startPose)
                .back(firstBackwardDistance)
                .build();

        Trajectory secondForward = mainRobot.trajectoryBuilder(firstBack.end())
                .forward(60)
                .build();

        Trajectory wiggle1 = mainRobot.trajectoryBuilder(secondForward.end())
                .splineToLinearHeading(new Pose2d(secondForward.end().getX()+1,
                        secondForward.end().getY(), -0.5),0,
                        SampleMecanumDrive.getVelocityConstraint(10,1, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .build();

        Trajectory wiggle2 = mainRobot.trajectoryBuilder(wiggle1.end())
                .splineToLinearHeading(new Pose2d(secondForward.end().getX()+2,
                        secondForward.end().getY(), 0.5),0,
                        SampleMecanumDrive.getVelocityConstraint(10,1, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .build();

        Trajectory wiggle3 = mainRobot.trajectoryBuilder(wiggle2.end())
                .splineToLinearHeading(secondForward.end(),0,
                        SampleMecanumDrive.getVelocityConstraint(10,1, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .build();

        Trajectory postIntakeAdjust = mainRobot.trajectoryBuilder(wiggle3.end())
                .strafeLeft(1)
                .build();

        Trajectory thirdBack = mainRobot.trajectoryBuilder(postIntakeAdjust.end())
                .back(60)
                .build();

        Trajectory fourthForward = mainRobot.trajectoryBuilder(thirdBack.end())
                .forward(60)
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

        mainRobot.followTrajectory(firstBack);
        mainRobot.pause(300);

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
        mainRobot.pause(500);

        mainRobot.extender.closeReleaser();
        mainRobot.pause(200);
        mainRobot.extender.setExtenderPower(-0.8);
        mainRobot.pause(400);
        while(mainRobot.extender.inSwitch.getState()) {
            mainRobot.extender.rotator.setPosition(0.32);
        }
        mainRobot.extender.setExtenderPower(0);
        mainRobot.intake.rightSlapOut();
        mainRobot.intake.leftSlapOut();

        mainRobot.pause(500);

        mainRobot.followTrajectory(secondForward);

        mainRobot.pause(500);

        mainRobot.intake.setLeftPower(-1);
        boolean gotOne = false;
        while(!gotOne) {
            mainRobot.flushLeftIntake();
            mainRobot.pause(100);
            if(mainRobot.leftFlushing) break;
            mainRobot.followTrajectory(wiggle1);
            mainRobot.flushLeftIntake();
            mainRobot.pause(100);
            if(mainRobot.leftFlushing) break;
            mainRobot.followTrajectory(wiggle2);
            mainRobot.flushLeftIntake();
            mainRobot.pause(100);
            if(mainRobot.leftFlushing) break;
            mainRobot.followTrajectory(wiggle3);
            mainRobot.pause(200);
        }
        telemetry.addData("WE GOIN", "YEEHAW");
        telemetry.update();
        mainRobot.pause(200);
        mainRobot.extender.closeReleaser();
        mainRobot.pause(200);

        mainRobot.followTrajectory(mainRobot.trajectoryBuilder(mainRobot.getPoseEstimate())
                    .splineToLinearHeading(secondForward.end(),0,
                        SampleMecanumDrive.getVelocityConstraint(10,1, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                    .build());
        mainRobot.pause(200);

        mainRobot.followTrajectory(postIntakeAdjust);
        mainRobot.pause(200);

        mainRobot.followTrajectory(thirdBack);
        mainRobot.pause(500);
        level = 3;

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
        mainRobot.pause(500);

        mainRobot.extender.closeReleaser();
        mainRobot.pause(200);
        mainRobot.extender.setExtenderPower(-0.8);
        mainRobot.pause(400);
        while(mainRobot.extender.inSwitch.getState()) {
            mainRobot.extender.rotator.setPosition(0.32);
        }
        mainRobot.extender.setExtenderPower(0);

        mainRobot.pause(500);

        mainRobot.followTrajectory(fourthForward);

        mainRobot.pause(500);


    }

}

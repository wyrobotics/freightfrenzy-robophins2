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
public class RedCycle extends LinearOpMode {

    MainRobot mainRobot;

    public static double firstBackwardDistance = 24;
    public static long firstExtend = 400;
    public static double firstExtendPower = 0.8;
    public static double raiseInc = 0.45 / 3;
    public static long raiseTime = 500;
    public static double[] raise = new double[] {0.0,0.5,0.62,0.72};
    //public static long secondExtend = 200;
    public static long[] secondExtend = new long[] {0,500,400,470};
    public static double secondExtendPower = 0.8;
    public static long releaseTime = 1000;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        Pose2d startPose = mainRobot.getPoseEstimate();

        Trajectory firstForward = mainRobot.trajectoryBuilder(startPose)
                .forward(firstBackwardDistance)
                .build();

        Trajectory secondBack = mainRobot.trajectoryBuilder(firstForward.end())
                .back(61)
                .build();

        Trajectory wiggle1 = mainRobot.trajectoryBuilder(secondBack.end())
                .splineToLinearHeading(new Pose2d(secondBack.end().getX()-2,
                                secondBack.end().getY(), -0.5),0,
                        SampleMecanumDrive.getVelocityConstraint(10,1, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .build();

        Trajectory wiggle2 = mainRobot.trajectoryBuilder(wiggle1.end())
                .splineToLinearHeading(new Pose2d(secondBack.end().getX()-4,
                                secondBack.end().getY(), 0.5),0,
                        SampleMecanumDrive.getVelocityConstraint(10,1, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .build();

        Trajectory wiggle3 = mainRobot.trajectoryBuilder(wiggle2.end())
                .splineToLinearHeading(secondBack.end(),0,
                        SampleMecanumDrive.getVelocityConstraint(10,1, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .build();

        Trajectory postIntakeAdjust = mainRobot.trajectoryBuilder(wiggle3.end())
                .strafeLeft(1)
                .build();

        Trajectory thirdForward = mainRobot.trajectoryBuilder(postIntakeAdjust.end())
                .forward(60)
                .build();

        Trajectory fourthBack = mainRobot.trajectoryBuilder(thirdForward.end())
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

        mainRobot.followTrajectory(firstForward);
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
        mainRobot.extender.openReleaser();

        mainRobot.pause(500);

        mainRobot.followTrajectory(secondBack);

        mainRobot.pause(500);

        mainRobot.intake.setRightPower(1);
        boolean gotOne = false;
        while(!gotOne) {
            mainRobot.flushRightIntake();
            mainRobot.pause(100);
            if(mainRobot.rightFlushing) break;
            mainRobot.followTrajectory(wiggle1);
            mainRobot.flushRightIntake();
            mainRobot.pause(100);
            if(mainRobot.rightFlushing) break;
            mainRobot.followTrajectory(wiggle2);
            mainRobot.flushRightIntake();
            mainRobot.pause(100);
            if(mainRobot.rightFlushing) break;
            mainRobot.followTrajectory(wiggle3);
            mainRobot.pause(200);
        }
        telemetry.addData("WE GOIN", "YEEHAW");
        telemetry.update();
        mainRobot.pause(1000);
        mainRobot.extender.closeReleaser();
        mainRobot.pause(200);

        mainRobot.followTrajectory(mainRobot.trajectoryBuilder(mainRobot.getPoseEstimate())
                .splineToLinearHeading(secondBack.end(),0,
                        SampleMecanumDrive.getVelocityConstraint(10,1, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .build());
        mainRobot.pause(200);

        mainRobot.followTrajectory(postIntakeAdjust);
        mainRobot.pause(200);

        mainRobot.followTrajectory(thirdForward);
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

        mainRobot.followTrajectory(fourthBack);

        mainRobot.pause(500);


    }

}

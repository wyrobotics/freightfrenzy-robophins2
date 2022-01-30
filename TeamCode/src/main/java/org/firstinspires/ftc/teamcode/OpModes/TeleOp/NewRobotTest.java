package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

@TeleOp
public class NewRobotTest extends LinearOpMode {

    MainRobot mainRobot;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {

            double stickX = gamepad1.left_stick_x, stickY = gamepad1.left_stick_y;
            mainRobot.setWeightedDrivePower(new Pose2d(-stickY, -stickX, -gamepad1.right_stick_x));

            if (gamepad1.right_bumper) {
                mainRobot.intake.setRightPower(1);
            } else if (gamepad1.right_trigger > 0.05) {
                mainRobot.intake.setRightPower(-1);
            } else {
                mainRobot.intake.setRightPower(0);
            }

            if (gamepad1.left_bumper) {
                mainRobot.intake.setLeftPower(1);
            } else if (gamepad1.left_trigger > 0.05) {
                mainRobot.intake.setLeftPower(-1);
            } else {
                mainRobot.intake.setLeftPower(0);
            }

            if(gamepad1.a) {
                mainRobot.intake.rightSlapIn();
                mainRobot.intake.leftSlapIn();
            }
            if(gamepad1.b) {
                mainRobot.intake.leftSlapOut();
                mainRobot.intake.rightSlapOut();
            }

        }

    }

}

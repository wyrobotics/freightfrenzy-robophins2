package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

@TeleOp
public class NewTeleOp extends LinearOpMode {

    MainRobot mainRobot;

    @Override
    public void runOpMode() {

        boolean fieldCentric = false;
        boolean back = false;

        mainRobot = new MainRobot(hardwareMap, telemetry);

        boolean dpadLeft = false;
        boolean dpadRight = false;
        waitForStart();
        while(opModeIsActive()) {

            //PLAYER 1

            if(gamepad1.y) {
                mainRobot.setPoseEstimate(new Pose2d(0,0,0));
            }

            if(gamepad1.back && !back) {
                back = true;
                fieldCentric = !fieldCentric;
            } else if(!gamepad1.back && back) {
                back = false;
            }

            double heading = mainRobot.getPoseEstimate().getHeading();

            double stickX = gamepad1.left_stick_x, stickY = -gamepad1.left_stick_y;

            if(fieldCentric) {

                mainRobot.setWeightedDrivePower(new Pose2d(
                    -((stickX * Math.sin(heading)) + (stickY * Math.cos(heading))),
                    -((stickX * Math.cos(heading)) - (stickY * Math.sin(heading))),
                    gamepad1.right_stick_x));

            } else {
                mainRobot.setWeightedDrivePower(new Pose2d(-stickY, -stickX, -gamepad1.right_stick_x));
            }

            if (gamepad1.right_bumper) mainRobot.intake.setRightPower(1);
            else if (gamepad1.right_trigger > 0.05) mainRobot.intake.setRightPower(-1);
            else mainRobot.intake.setRightPower(0);


            if (gamepad1.left_bumper) mainRobot.intake.setLeftPower(-1);
            else if (gamepad1.left_trigger > 0.05) mainRobot.intake.setLeftPower(1);
            else mainRobot.intake.setLeftPower(0);

            if(gamepad1.a) {
                mainRobot.intake.rightSlapIn();
                mainRobot.intake.leftSlapIn();
            }
            if(gamepad1.b) {
                mainRobot.intake.rightSlapOut();
                mainRobot.intake.leftSlapOut();
            }




            //PLAYER 2

            if(gamepad2.right_trigger > 0.05) mainRobot.spinner.spin(gamepad2.right_trigger);
            else if(gamepad2.left_trigger > 0.05) mainRobot.spinner.spin(-gamepad2.left_trigger);
            else if(gamepad2.right_bumper) mainRobot.spinner.spin(0.8);
            else if(gamepad2.left_bumper) mainRobot.spinner.spin(-0.8);
            else mainRobot.spinner.spin(0);

            if(gamepad2.a) mainRobot.extender.openReleaser();
            else if(gamepad2.b) mainRobot.extender.closeReleaser();

            if(gamepad2.dpad_down) mainRobot.extender.increaseRotatorPosition();
            if(gamepad2.dpad_up) mainRobot.extender.decreaseRotatorPosition();
            if(gamepad2.dpad_left) mainRobot.extender.setExtenderPower(-1);
            else if(gamepad2.dpad_right) mainRobot.extender.setExtenderPower(1);
            else mainRobot.extender.setExtenderPower(0);

            //telemetry.addData("Rotator: ", mainRobot.extender.rotatorPosition);
            //telemetry.addData("Up Switch Pressed? : ", mainRobot.lifter.upSwitch.isPressed());
            telemetry.addData("In switch: ", mainRobot.extender.inSwitch.isPressed());
            telemetry.addData("Field centric? ", fieldCentric);
            telemetry.addData("X: ", mainRobot.getPoseEstimate().getX());
            telemetry.addData("Y: ", mainRobot.getPoseEstimate().getY());
            telemetry.addData("Heading: ", mainRobot.getPoseEstimate().getHeading());
            telemetry.addData("Rotator: ", mainRobot.extender.rotatorPosition);
            telemetry.update();

        }

    }

}

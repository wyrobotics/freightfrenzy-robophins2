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

        boolean bPressed = false;
        boolean aPressed = false;
        boolean grabberOpen = true;

        boolean armLayer = true;
        boolean spinnerLayer = false;

        boolean dpadLeft = false;
        boolean dpadRight = false;
        boolean player2y = false;
        boolean player1y = false;

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {

            //PLAYER 1

            /*
            if(gamepad1.y) {
                mainRobot.setPoseEstimate(new Pose2d(0,0,0));
            }

            if(gamepad1.back && !back) {
                back = true;
                fieldCentric = !fieldCentric;
            } else if(!gamepad1.back && back) {
                back = false;
            }
             */

            double heading = mainRobot.getPoseEstimate().getHeading();

            double stickX = -gamepad1.left_stick_y, stickY = gamepad1.left_stick_x;

            if(fieldCentric) {

                mainRobot.setWeightedDrivePower(new Pose2d(
                    -((stickX * Math.sin(heading)) + (stickY * Math.cos(heading))),
                    -((stickX * Math.cos(heading)) - (stickY * Math.sin(heading))),
                    gamepad1.right_stick_x));

            } else {
                mainRobot.setWeightedDrivePower(new Pose2d(-stickY, -stickX, -gamepad1.right_stick_x));
            }



            /*
            if(gamepad1.a) {
                mainRobot.intake.rightSlapIn();
                mainRobot.intake.leftSlapIn();
            }
            if(gamepad1.b) {
                mainRobot.intake.rightSlapOut();
                mainRobot.intake.leftSlapOut();
            }
             */

            if(!gamepad1.a) {
                mainRobot.intake.leftSlapIn();
                mainRobot.intake.rightSlapIn();
            } else {
                mainRobot.intake.leftSlapOut();
                mainRobot.intake.rightSlapOut();
            }

            if(!bPressed && gamepad1.b) {
                bPressed = true;
                if(grabberOpen) {
                    mainRobot.extender.closeReleaser();
                    grabberOpen = false;
                }
                else {
                    mainRobot.extender.openReleaser();
                    grabberOpen = true;
                }
            }
            if(bPressed && !gamepad1.b)
                bPressed = false;

            if(gamepad1.y && !player1y) {
                player1y = true;
                spinnerLayer = !spinnerLayer;
            }
            if(!gamepad1.y && player1y) player1y = false;

            if(spinnerLayer) {
                if(gamepad1.right_trigger > 0.05) mainRobot.spinner.spin(gamepad1.right_trigger);
                else if(gamepad1.left_trigger > 0.05) mainRobot.spinner.spin(-gamepad1.left_trigger);
                else if(gamepad1.right_bumper) mainRobot.spinner.spin(0.8);
                else if(gamepad1.left_bumper) mainRobot.spinner.spin(-0.8);
                else mainRobot.spinner.spin(0);

                mainRobot.intake.setRightPower(0);
                mainRobot.intake.setLeftPower(0);
            } else {
                if (gamepad1.right_bumper) mainRobot.intake.setRightPower(1);
                else if (gamepad1.right_trigger > 0.05) mainRobot.intake.setRightPower(-1);
                else mainRobot.intake.setRightPower(0);

                if (gamepad1.left_bumper) mainRobot.intake.setLeftPower(-1);
                else if (gamepad1.left_trigger > 0.05) mainRobot.intake.setLeftPower(1);
                else mainRobot.intake.setLeftPower(0);

                mainRobot.spinner.spin(0);
            }





            //PLAYER 2



            /*
            if(gamepad2.a) mainRobot.extender.openReleaser();
            else if(gamepad2.b && !gamepad2.start) mainRobot.extender.closeReleaser();
             */

            /*
            if(gamepad2.dpad_down) mainRobot.extender.increaseRotatorPosition();
            if(gamepad2.dpad_up) mainRobot.extender.decreaseRotatorPosition();
            if(gamepad2.dpad_left) mainRobot.extender.setExtenderPower(-1);
            else if(gamepad2.dpad_right) mainRobot.extender.setExtenderPower(1);
            else mainRobot.extender.setExtenderPower(0);
             */

            /*
            mainRobot.shooter.yassify(-gamepad2.left_stick_x);
            mainRobot.shooter.pitchy(gamepad2.left_stick_y);
            mainRobot.shooter.shoot(gamepad2.right_stick_y);
             */

            if(gamepad2.y && !player2y) {
                player2y = true;
                armLayer = !armLayer;
            }
            if(!gamepad2.y && player2y) player2y = false;

            if(!armLayer) {
                mainRobot.shooter.yassify(-gamepad2.left_stick_x);
                mainRobot.shooter.pitchy(gamepad2.left_stick_y);
                mainRobot.shooter.shoot(gamepad2.right_stick_y);
                mainRobot.extender.setExtenderPower(0);
            } else {
                mainRobot.extender.setExtenderPower(-gamepad2.left_stick_y);
                mainRobot.extender.changeRotatorPosition(gamepad2.right_stick_y / 150.0);
                mainRobot.shooter.yassify(0);
                mainRobot.shooter.pitchy(0);
                mainRobot.shooter.shoot(0);
            }

            if(!aPressed && gamepad2.a) {
                aPressed = true;
                if(grabberOpen) {
                    mainRobot.extender.closeReleaser();
                    grabberOpen = false;
                }
                else {
                    mainRobot.extender.openReleaser();
                    grabberOpen = true;
                }
            }
            if(aPressed && !gamepad2.a)
                aPressed = false;


            //telemetry.addData("Rotator: ", mainRobot.extender.rotatorPosition);
            //telemetry.addData("Up Switch Pressed? : ", mainRobot.lifter.upSwitch.isPressed());
            telemetry.addData("In switch: ", mainRobot.extender.inSwitch.getState());
            telemetry.addData("Field centric? ", fieldCentric);
            telemetry.addData("X: ", mainRobot.getPoseEstimate().getX());
            telemetry.addData("Y: ", mainRobot.getPoseEstimate().getY());
            telemetry.addData("Heading: ", mainRobot.getPoseEstimate().getHeading());
            telemetry.addData("Rotator: ", mainRobot.extender.rotatorPosition);
            telemetry.update();

        }

    }

}

package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

@TeleOp
public class MotorTest extends LinearOpMode {

    MainRobot mainRobot;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {

            if(gamepad1.a) mainRobot.leftRear.setPower(1);
            else mainRobot.leftRear.setPower(0);
            if(gamepad1.b) mainRobot.rightRear.setPower(1);
            else mainRobot.rightRear.setPower(0);
            if(gamepad1.y) mainRobot.rightFront.setPower(1);
            else mainRobot.rightFront.setPower(0);
            if(gamepad1.x) mainRobot.leftFront.setPower(1);
            else mainRobot.leftFront.setPower(0);

        }

    }

}

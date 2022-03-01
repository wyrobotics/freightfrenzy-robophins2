package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

@TeleOp
public class ColorSensorTest extends LinearOpMode {

    MainRobot mainRobot;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {

            telemetry.addData("Right Sensor: ", mainRobot.intake.getRightColor().print());
            telemetry.addData("Left Sensor: ", mainRobot.intake.getLeftColor().print());
            telemetry.update();

        }

    }

}
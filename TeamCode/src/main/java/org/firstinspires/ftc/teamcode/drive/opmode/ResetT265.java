package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Hardware.T265Static;
import org.firstinspires.ftc.teamcode.Components.MainRobot;

@TeleOp
public class ResetT265 extends LinearOpMode {

    MainRobot mainRobot;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        T265Static.slamera.stop();
        T265Static.slamera = null;

    }

}

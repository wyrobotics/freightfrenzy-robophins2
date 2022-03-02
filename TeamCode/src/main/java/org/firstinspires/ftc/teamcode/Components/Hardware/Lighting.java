package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lighting {

    RevBlinkinLedDriver ledDriver;

    public Lighting(HardwareMap hardwareMap, Telemetry telemetry) {

        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledDriver");

    }

    public void signalFreightReceived() { ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN); }
    public void signalNoFreight() { ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED); }
    public void signalAutoIntakeOff() { ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW); }

}

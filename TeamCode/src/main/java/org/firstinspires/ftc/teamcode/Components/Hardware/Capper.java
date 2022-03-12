package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Capper {

    Servo capper;
    Servo actuator;

    public static double minPos = 0.0;
    public static double maxPos = 1.0;

    public static double openPos = 0.9;
    public static double closePos = 0.45;

    public double capPos = minPos;

    public Capper(HardwareMap hardwareMap, Telemetry telemetry) {

        capper = hardwareMap.get(Servo.class, "capper");
        actuator = hardwareMap.get(Servo.class, "pitch");

    }

    public void changeCapperPos(double d) {
        capPos = Math.min(maxPos, Math.max(minPos, capPos + d));
        capper.setPosition(capPos);
    }

    public void setCapperPos(double pos) {
        capPos = Math.min(maxPos, Math.max(minPos, pos));
        capper.setPosition(capPos);
    }

    public void closeActuator() { actuator.setPosition(closePos); }
    public void openActuator() { actuator.setPosition(openPos); }

}

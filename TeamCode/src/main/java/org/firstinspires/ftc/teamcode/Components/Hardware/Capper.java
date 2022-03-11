package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Capper {

    Servo capper;

    public static double minPos = 0.0;
    public static double maxPos = 1.0;

    public double capPos = minPos;

    public Capper(HardwareMap hardwareMap, Telemetry telemetry) {

        capper = hardwareMap.get(Servo.class, "capper");

    }

    public void changeCapperPos(double d) {
        capPos = Math.min(maxPos, Math.max(minPos, capPos + d));
        capper.setPosition(capPos);
    }

    public void setCapperPos(double pos) {
        capPos = Math.min(maxPos, Math.max(minPos, pos));
        capper.setPosition(capPos);
    }

}

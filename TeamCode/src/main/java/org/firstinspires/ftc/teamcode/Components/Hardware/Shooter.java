package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Shooter {

    CRServo yaw;
    Servo pitch;
    DcMotor shooter;

    public static double maxPitch = 1.0;
    public static double minPitch = 0.0;
    public static double pitchPos = 0.5;

    Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {

        yaw = hardwareMap.get(CRServo.class, "yaw");
        pitch = hardwareMap.get(Servo.class, "pitch");
        //pitch.scaleRange(0.0, 0.66);

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        this.telemetry = telemetry;

    }

    public void shoot(double power) { shooter.setPower(power); }

    public void yassify(double power) {
        //if(Math.abs(power) > 0.2) yaw.setPower(power);
        //else yaw.setPower(0);
        yaw.setPower(power * 0.2);
    }

    public void pitchy(double power) {
        pitchPos = Math.max(minPitch, Math.min(maxPitch, pitchPos - (power/150.0)));
        telemetry.addData("Pitch: ", pitchPos);
        pitch.setPosition(pitchPos);
    }

}

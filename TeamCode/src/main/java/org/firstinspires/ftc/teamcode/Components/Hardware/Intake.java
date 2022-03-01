package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    DcMotor leftIntake;
    DcMotor rightIntake;
    Servo leftIntakeSlapper;
    Servo rightIntakeSlapper;

    ColorSensor rightSensor;
    ColorSensor leftSensor;

    public class Color {

        public double red, green, blue;

        public Color(double red, double green, double blue) {
            this.red = red; this.green = green; this.blue = blue;
        }

        public String print() { return red + " " + green + " " + blue; }

    }

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntakeSlapper = hardwareMap.get(Servo.class, "leftIntakeSlapper");
        rightIntakeSlapper = hardwareMap.get(Servo.class, "rightIntakeSlapper");

        rightSensor = hardwareMap.get(ColorSensor.class, "rightSensor");
        leftSensor = hardwareMap.get(ColorSensor.class, "leftSensor");

    }

    public void setLeftPower(double power) {
        leftIntake.setPower(power);
    }

    public void setRightPower(double power) {
        rightIntake.setPower(power);
    }

    public void leftSlap(double pos) { leftIntakeSlapper.setPosition(pos); }
    public void leftSlapIn() { leftSlap(0.45); }
    public void leftSlapOut() { leftSlap(1); }

    public void rightSlap(double pos) { rightIntakeSlapper.setPosition(pos); }
    public void rightSlapIn() { rightSlap(0.55); }
    public void rightSlapOut() { rightSlap(0.0); }

    public Color getRightColor() { return new Color(rightSensor.red(), rightSensor.green(), rightSensor.blue()); }
    public Color getLeftColor() { return new Color(leftSensor.red(), leftSensor.green(), leftSensor.blue()); }

}

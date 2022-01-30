package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    DcMotor leftIntake;
    DcMotor rightIntake;
    Servo leftIntakeSlapper;
    Servo rightIntakeSlapper;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntakeSlapper = hardwareMap.get(Servo.class, "leftIntakeSlapper");
        rightIntakeSlapper = hardwareMap.get(Servo.class, "rightIntakeSlapper");

    }

    public void setLeftPower(double power) {
        leftIntake.setPower(power);
    }

    public void setRightPower(double power) {
        rightIntake.setPower(power);
    }

    public void leftSlap(double pos) { leftIntakeSlapper.setPosition(pos); }
    public void leftSlapIn() { leftSlap(0.725); }
    public void leftSlapOut() { leftSlap(1); }

    public void rightSlap(double pos) { rightIntakeSlapper.setPosition(pos); }
    public void rightSlapIn() { rightSlap(0.3); }
    public void rightSlapOut() { rightSlap(0.0); }

}

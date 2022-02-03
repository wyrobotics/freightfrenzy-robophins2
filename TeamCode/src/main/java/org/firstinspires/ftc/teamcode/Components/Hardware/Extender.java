package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Extender {

    private DcMotor extender;
    private Servo rotator;
    private Servo releaser;

    public DigitalChannel inSwitch;

    private TouchSensor outSwitch;

    static public double maxRotatorPosition = 0.9;
    public double rotatorPosition = maxRotatorPosition;

    public Extender(HardwareMap hardwareMap, Telemetry telemetry) {

        extender = hardwareMap.get(DcMotor.class, "extender");
        extender.setDirection(DcMotorSimple.Direction.REVERSE);
        rotator = hardwareMap.get(Servo.class, "rotator");
        releaser = hardwareMap.get(Servo.class, "releaser");

        inSwitch = hardwareMap.get(DigitalChannel.class, "inSwitch");
        //outSwitch = hardwareMap.get(TouchSensor.class, "outSwitch");
        inSwitch.setMode(DigitalChannel.Mode.INPUT);

        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //rotator.scaleRange(0.5, 0.95);

        rotator.setPosition(rotatorPosition);

    }


    public void setExtenderPower(double power) {

        double finalPower = power;

        if(!inSwitch.getState()) { finalPower = Math.max(0, finalPower); }
        //if(outSwitch.getState()) { finalPower = Math.min(0, finalPower); }

        extender.setPower(finalPower);
    }

    public void increaseRotatorPosition() {
        //rotator.setPosition(rotator.getPosition() + 0.00001);
        rotatorPosition = Math.min(maxRotatorPosition, rotatorPosition + 0.005);
        rotator.setPosition(rotatorPosition);
    }
    public void decreaseRotatorPosition() {
        //rotator.setPosition(rotator.getPosition() - 0.00001);
        rotatorPosition = Math.min(rotatorPosition, rotatorPosition - 0.005);
        rotator.setPosition(rotatorPosition);
    }

    public void openReleaser() { releaser.setPosition(0.95); }
    public void closeReleaser() { releaser.setPosition(0.72); }

}

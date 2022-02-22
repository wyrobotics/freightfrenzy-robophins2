package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.MainRobot;
import org.firstinspires.ftc.teamcode.Components.Software.OpenCVSampler;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp
public class OpenCVTest extends LinearOpMode {

    OpenCVSampler openCVSampler;
    OpenCvCamera webcam;

    @Override
    public void runOpMode() {

        openCVSampler = new OpenCVSampler(hardwareMap, telemetry);
        webcam = openCVSampler.camera;

        openCVSampler.initOpenCVCamera();

        //FtcDashboard.getInstance().startCameraStream();

        waitForStart();
        while(opModeIsActive()) {

            telemetry.addData("Pink: ", openCVSampler.getPinkX());
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

        }

    }

}

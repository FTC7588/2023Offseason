package org.firstinspires.ftc.teamcode.opmode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.PixelProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp
public class Bare extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        PixelProcessor pixelProcessor = new PixelProcessor();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(pixelProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableCameraMonitoring(true)
                .setAutoStopLiveView(false)
                .build();

        telemetry.addLine("Waiting:");
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

//            ArrayList<AprilTagDetection> tagList = tagProcessor.getDetections();

            //telemetry.addData("# tags", tagList.size());

            telemetry.addLine("running");

            telemetry.update();

        }

    }

}

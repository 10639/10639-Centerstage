package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.PropDetection;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "VisionTest", preselectTeleOp = "CenterStage_TeleOp")
public class VisionTest extends LinearOpMode {

    public SampleMecanumDrive driveTrain;
    public PIDController controller;
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new SampleMecanumDrive(hardwareMap);
        PropDetection pipeline;
        OpenCvWebcam webcam;

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
                );

        webcam = OpenCvCameraFactory
                .getInstance().createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam"),
                        cameraMonitorViewId);

        pipeline = new PropDetection(telemetry, true); //Blue -> False, Red -> True
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        PropDetection.Location location = null;
        while (!isStarted()) {
            location = pipeline.getLocation();
            telemetry.addData("Location", location);
            telemetry.update();
        }

    }
}
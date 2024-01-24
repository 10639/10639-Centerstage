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

@Autonomous(name = "Auto_BlueLeft", preselectTeleOp = "CenterStage_TeleOp")
public class Auto_BlueLeft extends LinearOpMode {

    public SampleMecanumDrive driveTrain;
    public PIDController controller;
    Lift liftSystem;
    Arm armSystem;
    Intake intakeSystem;
    OpenCvCamera camera;
    //  Detection detectionSystem;


    Pose2d initPose;
    Vector2d leftLocation, centerLocation, rightLocation;
    Vector2d midwayVector;
    Vector2d scoringVector;
    Vector2d leftVector;
    Vector2d rightVector;
    Vector2d finalPose;
    Vector2d parkingPose;
   // Vector2d parkingLocationOne, parkingLocationTwo, parkingLocationThree;

    public enum TrajectoryState {PRELOAD, CS, CS_TO_MJ, PARK, IDLE}

    TrajectoryState trajectoryState = TrajectoryState.PRELOAD;

    public static int identifiedLocation = 2;
    public static int targetPosition = 0;
    public boolean preloadDone = false;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new SampleMecanumDrive(hardwareMap);
        liftSystem = new Lift(hardwareMap);
        armSystem = new Arm(hardwareMap);
        intakeSystem = new Intake(hardwareMap);
      //  detectionSystem = new Detection(hardwareMap);
        //ElapsedTime time = new ElapsedTime();

      //  liftSystem.init();
        armSystem.init();
        intakeSystem.init();
        int cameraMonitorViewId;
         cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
            FtcDashboard.getInstance().startCameraStream(camera, 0);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                             @Override
                                             public void onOpened() {
                                                 camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                                             }

                                             @Override
                                             public void onError(int errorCode) {

                                             }
                                         });
      //  detectionSystem.init(bluePipeline);




       // initPose = new Pose2d(-35, -58, Math.toRadians(270));
        initPose = new Pose2d(13, 58, Math.toRadians(-270));
        midwayVector = new Vector2d(13, 33);
        scoringVector = new Vector2d(47, 36);
        parkingPose = new Vector2d(47,56);
        finalPose = new Vector2d(60, 56);
    //    midWayPose = new Vector2d(-35, -10);
        //  dropConePose = new Vector2d(-29, 28);



        TrajectorySequence centerPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(midwayVector)
                .back(-3)
                .turn(Math.toRadians(90))
                .lineToConstantHeading(scoringVector)
                .waitSeconds(3)
                .strafeTo(parkingPose)
                .lineToConstantHeading(finalPose)
                .build();

        TrajectorySequence rightPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(midwayVector)
                .strafeTo(leftVector)
                .back(-3)
                .strafeTo(new Vector2d(13, 36))
                .turn(Math.toRadians(90))
                .lineToConstantHeading(scoringVector)
                .strafeTo(new Vector2d(47, 28))
                .waitSeconds(3)
                .strafeTo(parkingPose)
                .lineToConstantHeading(finalPose)
                .build();

        TrajectorySequence leftPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(midwayVector)
                .strafeTo(rightVector)
                .back(-3)
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(35, 36))
                .lineToConstantHeading(scoringVector)
                .strafeTo(new Vector2d(47, 42))
                .waitSeconds(3)
                .strafeTo(new Vector2d(47,55))
                .strafeTo(parkingPose)
                .lineToConstantHeading(finalPose)
                .build();




        driveTrain.getLocalizer().setPoseEstimate(initPose);
        while (!isStopRequested() && !opModeIsActive()) {
            //identifiedLocation = detectionSystem.detect();
         //   telemetry.addData("Detected Parking Location:", identifiedLocation);
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        waitForStart();
        driveTrain.followTrajectorySequenceAsync(centerPreload);
    //    detectionSystem.stop();

        while (!isStopRequested()) {
            while (opModeIsActive()) {
                driveTrain.update();

               /*
                liftSystem.controller.setPID(Constants.Kp, Constants.Ki, Constants.Kd);
                int state = liftSystem.rightSlide.getCurrentPosition();
                telemetry.addData("state", state);
                telemetry.addData("target", targetPosition);
                telemetry.update();
                double pid = liftSystem.controller.calculate(state, targetPosition);
                double power = pid + Constants.Kf;
                liftSystem.leftSlide.setPower(power * 0.8);
                liftSystem.rightSlide.setPower(power * 0.8);

//                switch (identifiedLocation) {
//                    case 1:
//                        if (!driveTrain.isBusy()) {
//                            driveTrain.followTrajectorySequenceAsync(locationOne);
//                            trajectoryState = TrajectoryState.IDLE;
//                        }
//                        break;
//                    case 2:
//                        if (!driveTrain.isBusy()) {
//                            driveTrain.followTrajectorySequenceAsync(locationTwo);
//                            trajectoryState = TrajectoryState.IDLE;
//                        }
//
//                    case 3:
//                        if (!driveTrain.isBusy()) {
//                            driveTrain.followTrajectorySequenceAsync(locationThree);
//                            trajectoryState = TrajectoryState.IDLE;
//                        }
//                        break;
//                }

                switch (trajectoryState) {
                    case PRELOAD:
                        targetPosition = Constants.LIFT_LEVEL_ZERO;
                        if (!driveTrain.isBusy()) {
                            trajectoryState = TrajectoryState.PARK;
                        }
                        break;
                    case PARK:
                        targetPosition = Constants.LIFT_LEVEL_ZERO;
                        switch (identifiedLocation) {
                            case 1:
                                if (!driveTrain.isBusy()) {
                                    driveTrain.followTrajectorySequenceAsync(locationOne);
                                    trajectoryState = TrajectoryState.IDLE;
                                }
                                break;
                            case 2:
                                if (!driveTrain.isBusy()) {
                                    driveTrain.followTrajectorySequenceAsync(locationTwo);
                                    trajectoryState = TrajectoryState.IDLE;
                                }
                                break;
                            case 3:
                                if (!driveTrain.isBusy()) {
                                    driveTrain.followTrajectorySequenceAsync(locationThree);
                                    trajectoryState = TrajectoryState.IDLE;
                                }
                                break;
                        }
                        break;
                    case IDLE:
                        liftSystem.leftSlide.setTargetPosition(0);
                        liftSystem.leftSlide.setPower(1);
                        liftSystem.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        break;


                }

                */
            }
        }

    }

}





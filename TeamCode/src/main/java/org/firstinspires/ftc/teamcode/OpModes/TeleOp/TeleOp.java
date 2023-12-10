package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Control.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PowerPlay_TeleOp")
public class TeleOp extends LinearOpMode {

    public PIDController controller;
    public SampleMecanumDrive driveTrain;
   // public Arm armSystem;
  //  public Lift liftSystem;
    public ElapsedTime timer;
    public Servo claw;



    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new SampleMecanumDrive(hardwareMap);
        claw =  hardwareMap.get(Servo.class, "claw");
       // armSystem = new Arm(hardwareMap);
  //      liftSystem = new Lift(hardwareMap);
        timer = new ElapsedTime();

      //  liftSystem.init();
      //  armSystem.init();

        if (isStopRequested()) return;
        while (!isStopRequested()) {
            while (opModeIsActive()) {
                timer.reset();

                driveTrain.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

             //   armSystem.loop(gamepad2); // Continuously check to see if button was pressed to close and open claw
            ///    liftSystem.loop(gamepad2, armSystem);

                telemetry.addData("Loop Time: ", (1000 / timer.milliseconds()));
          //      liftSystem.showInfo(telemetry);
                telemetry.update();
            }
        }

    }

}

package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Control.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TESTING")
@Disabled
public class TeleOp_Test extends LinearOpMode {

  public PIDController controller;
  public DriveTrain driveTrain;
  public Arm armSystem;
  public ElapsedTime timer;
  public DcMotorEx liftMotor;
  public static int target = 0;


  @Override
  public void runOpMode() throws InterruptedException {
    controller = new PIDController(Constants.Kp, Constants.Ki, Constants.Kd);
    controller.setPID(Constants.Kp, Constants.Ki, Constants.Kd);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    liftMotor = hardwareMap.get(DcMotorEx.class, "elevator");
    liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    driveTrain = new DriveTrain(hardwareMap);
    armSystem = new Arm(hardwareMap);
    armSystem.init();

    if (isStopRequested()) return;
    while (!isStopRequested()) {
      while (opModeIsActive()) {

        driveTrain.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        armSystem.loop(gamepad2); // Continuously check to see if button was pressed to close and open claw


        if(gamepad2.square) {
          target = Constants.LIFT_FIRST_LEVEL;
        } else if(gamepad2.triangle) {
          target = Constants.LIFT_SECOND_LEVEL;
        } else if(gamepad2.circle) {
          target = Constants.LIFT_THIRD_LEVEL;
        } else if(gamepad2.cross) {
          target = Constants.LIFT_LEVEL_ZERO;
        } else if(gamepad2.dpad_up) {
          target = liftMotor.getCurrentPosition() + Constants.MANUAL_EXTEND_INCREMENT;
        } else if(gamepad2.dpad_down) {
          target = liftMotor.getCurrentPosition() - Constants.MANUAL_DESCEND_INCREMENT;
        }  else if(gamepad2.left_trigger > 0) {
          target = Constants.LIFT_LEVEL_ZERO;
        }

        int state = liftMotor.getCurrentPosition();
        double pid = controller.calculate(state, target);
        double power = pid + Constants.Kf;
        // liftMotor.setPower(state < target ? power : (0.2 * pid) + Constants.Kf);
        liftMotor.setPower(power);

        telemetry.addData("Pos", state);
        telemetry.addData("Target", target);
        telemetry.addData("power", liftMotor.getPower());
        telemetry.update();
      }
    }

  }

}

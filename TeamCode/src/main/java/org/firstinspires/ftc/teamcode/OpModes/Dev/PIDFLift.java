package org.firstinspires.ftc.teamcode.OpModes.Dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Constants;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PIDFLift_Test")
@Disabled
public class PIDFLift extends LinearOpMode {

    public PIDController controller;
    public DcMotorEx leftSlide, rightSlide;
    public static int target = 0;
    public static double p = 0, i = 0, d =0;
    public static double f = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);

        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);


        if (isStopRequested()) return;
        while (!isStopRequested()) {
            while (opModeIsActive()) {



                controller.setPID(p, i, d);
                int state = rightSlide.getCurrentPosition();
                double pid = controller.calculate(state, target);
                double power = pid + f;
               // liftMotor.setPower(state < target ? power : (0.2 * pid) + Constants.Kf);
                leftSlide.setPower(power * 0.8);
                rightSlide.setPower(power * 0.8);

                telemetry.addData("Pos", state);
                telemetry.addData("Target", target);
                telemetry.addData("power", power);
                telemetry.update();
            }
        }

    }

}

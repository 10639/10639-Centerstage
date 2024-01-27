package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PIDFLift_Test")
public class PIDFLift extends OpMode {

    private PIDController controller;
    public DcMotorEx leftSlide, rightSlide;
    public Arm armSystem;
    public static int target = 0;
    public static double p = 0.1, i = 0, d = 0.00001;
    //Possible Value for P: 0.1;
    //Possible Value for D: 0.00001;
    public static double f = 0.12; //Possible Value 0.12;
    public static boolean rightSlideRest = true;


    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        armSystem = new Arm(hardwareMap);
        armSystem.init();

        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {

        controller.setPID(p, i, d);
        int leftPosition = leftSlide.getCurrentPosition();
        double pid = controller.calculate(leftPosition, target);
        double power = pid + f;
        if (pid < 0) { // Going down
            power = Math.max(power, -0.15);
        } else { //Going up
            power = Math.min(power, 0.8); //Power Range 0 -> 1;
            if(target > 0) {
                rightSlideRest = false;
            }
        }
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        if (leftSlide.getCurrentPosition() > 15) {
            rightSlideRest = false;
        }
        if( (target == 0)  ) {
            while ((rightSlide.getCurrentPosition() > 1 || rightSlide.getCurrentPosition() <= -1) && !rightSlideRest) {
                rightSlide.setPower((Math.signum(rightSlide.getCurrentPosition() * -1) * 0.3));
                if (rightSlide.getCurrentPosition() < 1 || rightSlide.getCurrentPosition() >= -1) {
                    rightSlideRest = true;
                    rightSlide.setPower(0);
                    break;
                }
            }
               while(leftSlide.getCurrentPosition() > 0) {
                   leftSlide.setPower(-0.3);
                   if(leftSlide.getCurrentPosition() == 0) {
                       leftSlide.setPower(0);
                       break;
                   }
               }
        }

        telemetry.addData("leftPos", leftPosition);
        telemetry.addData("rightPos", rightSlide.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.addData("Calculated PID", pid);
        telemetry.addData("Slides Power", power);
        telemetry.addData("Slide Direction:", pid < 0 ? "Down" : "Up");
        telemetry.addData("Right Slide @ Rest", rightSlideRest);
        telemetry.update();

    }
}

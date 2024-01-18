package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servo_ArmTest")

public class Servo_ArmTest extends LinearOpMode {

    public static double min = 0;
    public static double max = 1;
    public static String state = "idle";
    public static Servo leftPivot, rightPivot;


    @Override
    public void runOpMode() throws InterruptedException {

        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");

        leftPivot.scaleRange(min, max);
        rightPivot.scaleRange(min, max);


        if (isStopRequested()) return;
        while (!isStopRequested()) {
            while (opModeIsActive()) {
                switch (state) {
                    case "idle":
                        setPosition(0, 1);
                        break;
                    case "score":
                        setPosition(1, 0);
                        break;
                }

                telemetry.addData("State", state);
                telemetry.addData("Min Position", min);
                telemetry.addData("Max Position", max);
                telemetry.update();


            }

        }

    }

    private static void setPosition(double left, double right) {
        leftPivot.setPosition(left);
        rightPivot.setPosition(right);
    }
}
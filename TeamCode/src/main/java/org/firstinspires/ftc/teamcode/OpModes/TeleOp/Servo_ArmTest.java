package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servo_ArmTest")

public class Servo_ArmTest extends LinearOpMode {

    public static double min = 0, max = 0;
    public static String state = "idle";
    public static Servo leftPivot, rightPivot;


    @Override
    public void runOpMode() throws InterruptedException {

        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        if (isStopRequested()) return;
        while (!isStopRequested()) {
            while (opModeIsActive()) {

                leftPivot.scaleRange(min, max);
                rightPivot.scaleRange(min, max);


                switch (state) {
                    case "idle":
                        leftPivot.setPosition(0);
                        rightPivot.setPosition(1);
                        break;
                    case "score":
                        leftPivot.setPosition(1);
                        rightPivot.setPosition(0);
                        break;
                }

                telemetry.addData("State", state);
                telemetry.addData("Min Position", min);
                telemetry.addData("Max Position", max);
                telemetry.update();


            }

        }

    }

}
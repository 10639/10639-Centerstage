package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Dev;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "pixelDetection")
public class pixelDetection extends LinearOpMode {

    private DistanceSensor sensorDistance;

    @Override
    public void runOpMode() {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("deviceName", sensorDistance.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }

}

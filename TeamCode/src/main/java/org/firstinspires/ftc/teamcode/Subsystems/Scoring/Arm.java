package org.firstinspires.ftc.teamcode.Subsystems.Scoring;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private final HardwareMap hardwareMap;
    public Servo leftPivot, rightPivot;
    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        leftPivot.scaleRange(0.6, 0.75);  //0 Pos becomes 0.3 -- 1 Pos becomes 0.8;
        armIdle();
    }

    public void armIdle() {
        leftPivot.setPosition(0);
    }

    public void armScore() {
        leftPivot.setPosition(1);
    }

    public void loop(Gamepad gamepad) {

         if(gamepad.triangle) {
             armScore();
         } else if(gamepad.cross) {
             armIdle();
         }

    }
}
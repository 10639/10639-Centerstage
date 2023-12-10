package org.firstinspires.ftc.teamcode.Subsystems.Scoring;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private final HardwareMap hardwareMap;
    public Servo claw, rotation, leftPivot, rightPivot;
    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        claw = hardwareMap.get(Servo.class, "claw");
        rotation = hardwareMap.get(Servo.class, "rotation");
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        closeArm(); //Close the arm to grab the preloaded
        initIdle(); // Put the arm slightly higher so when the drivetrain itself is backed up against the wall, the arm won't get stopped by the wall.
    }

    public void openArm() {
        claw.setPosition(Constants.openArm);
    }

    public void closeArm() {
        claw.setPosition(Constants.closeArm);
    }

    public void rotationScore() {
        rotation.setPosition(Constants.rotationScore);
    }

    public void rotationIdle() {
        rotation.setPosition(Constants.rotationIdle);
    }


    public void armIdle() {
        leftPivot.setPosition(Constants.leftArm_Idle);
        rightPivot.setPosition(Constants.rightArm_Idle);
    }

    public void armScore() {
        leftPivot.setPosition(Constants.leftArm_Score);
        rightPivot.setPosition(Constants.rightArm_Score);
    }
    public void initIdle() {
        closeArm();
        armIdle();
        rotationIdle();
        leftPivot.setPosition(0.5);
        rightPivot.setPosition(0.5);
    }

    public void scoreReady() {
        closeArm();
        rotationScore();
        armScore();
    }

    public void scoreIdle() {
        closeArm();
        armIdle();
        rotationIdle();
    }

    public void loop(Gamepad gamepad) {

        if (gamepad.left_bumper) {
            openArm();
        } else if (gamepad.right_bumper) {
            closeArm();
        } else if(gamepad.triangle) {
            scoreReady();
        } else if(gamepad.cross) {
            scoreIdle();
        }

    }
}
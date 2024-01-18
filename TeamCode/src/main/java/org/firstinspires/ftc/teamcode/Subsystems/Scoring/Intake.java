package org.firstinspires.ftc.teamcode.Subsystems.Scoring;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private final HardwareMap hardwareMap;
    public DcMotorEx sweeper;
    public CRServo boxSweeper;
    public Servo intake;
    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        boxSweeper = hardwareMap.get(CRServo.class ,"boxSweeper");
        intake = hardwareMap.get(Servo.class, "intake");
        sweeper = hardwareMap.get(DcMotorEx.class, "sweeper");
        sweeper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sweeper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sweeper.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sweeper.setDirection(DcMotor.Direction.REVERSE);
        intake.scaleRange(0, 0.23);
        initIdle();
        boxSweeper.setPower(0);
        intake.setPosition(0);
    }

//SWITCHED 1/13/24 MOTOR POSITIONS

    public void Sweep() { sweeper.setPower(Constants.Sweep * -1); }
    public void reverseSweep() { sweeper.setPower(Constants.Sweep); }
    public void terminateSweep() { sweeper.setPower(0); }


    public void initIdle() {
        terminateSweep();
    }

    public void loop(Gamepad gamepad) {

      if(gamepad.dpad_up) {
          Sweep();
      } else if(gamepad.dpad_down) {
          reverseSweep();
      } else if(gamepad.left_stick_button) {
          terminateSweep();
      } else if(gamepad.dpad_left) { //Reverses Box Spin
          boxSweeper.setPower(1);
      } else if(gamepad.dpad_right) { //Spins Pixels in
          boxSweeper.setPower(-1);
      } else if(gamepad.right_stick_button) { //Terminates Box spin
          boxSweeper.setPower(0);
      } else if(gamepad.left_bumper) { //Default V4b Position
          intake.setPosition(0);
      } else if(gamepad.right_bumper) { //Outer V4b Position
          intake.setPosition(1);
      }


    }
}


package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends Mechanism{
    public DcMotor intake;
    public Servo leftServo;
    public Servo rightServo;
    public Intake(DcMotor intake, Servo leftServo, Servo rightServo) {
         this.intake = intake;
         this.leftServo = leftServo;
         this.rightServo = rightServo;
         leftServo.setDirection(Servo.Direction.REVERSE);
    }
    public void runIntake(double power) {
        intake.setPower(power);
    }

    //valid parameters: up,down
    public void flipTransfer(String direction) {
        if (direction .equals("up")) {
            leftServo.setPosition(0);
            rightServo.setPosition(0);
        } else if (direction .equals("down")) {
            leftServo.setPosition(0.7);
            rightServo.setPosition(0.7);
        }
    }
    @Override
    public void update(double time) {
        //update later
    }
}

//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//
//public class PracticeIntake extends LinearOpMode{
//
//    private Servo rightServo = null;
//    private Servo leftServo = null;
//
//    @Override
//    public void runOpMode() {
//
//        rightServo = hardwareMap.get(Servo.class, "intake1");
//        leftServo = hardwareMap.get(Servo.class, "intake2");
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            handleIntakeControl();
//
//            private void handleIntakeControls() {
//                if (gamepad1.right_trigger > 0.1) {
//                    rightServo.setPower(gamepad1.right_trigger);
//                    leftServo.setPower(gamepad1.right_trigger);
//                }
//
//                else {
//                    rightServo.setPower(0,0);
//                    leftServo.setPower(0,0);
//                }
//            }
//
//        }
//
//
//
//    }
//}

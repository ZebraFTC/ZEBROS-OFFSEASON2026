package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class CodingSessionOne extends OpMode {
    private RevTouchSensor touchSensor;
    @Override
    public void init() {
        touchSensor = hardwareMap.get(RevTouchSensor.class, "button");

    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0) {
            telemetry.addData("trigger", true);
        }

    }
}

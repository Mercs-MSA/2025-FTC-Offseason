package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestbedSwyftDrive extends OpMode {

    private DcMotor swyftTest = null;
    private double x = 0.0;

    @Override
    public void init() {
        swyftTest = hardwareMap.get(DcMotor.class, "swyft");
    }

    @Override
    public void loop() {
        swyftTest.setPower(x);
        x = -gamepad1.left_stick_y;
    }
}

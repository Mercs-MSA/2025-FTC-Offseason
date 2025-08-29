package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestbedSwyftDrive extends OpMode {

    private DcMotor swyftTest = null;

    @Override
    public void init() {
        swyftTest = hardwareMap.get(DcMotor.class, "swyft");
    }

    @Override
    public void loop() {

    }
}

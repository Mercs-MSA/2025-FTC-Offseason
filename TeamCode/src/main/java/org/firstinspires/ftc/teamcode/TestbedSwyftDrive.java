package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class TestbedSwyftDrive extends OpMode {

    private DcMotorEx swyftTest = null;
    private double x = 0.0;

    @Override
    public void init() {
        swyftTest = hardwareMap.get(DcMotorEx.class, "swyft");

        swyftTest.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        x = -gamepad1.left_stick_y;
        swyftTest.setPower(x);

        telemetry.addData("Power:", swyftTest.getPower());
        telemetry.addData("Current:", swyftTest.getCurrent(CurrentUnit.AMPS));
    }
}

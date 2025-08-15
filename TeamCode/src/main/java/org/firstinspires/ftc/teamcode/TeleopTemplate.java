package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class TeleopTemplate extends OpMode {
    //Variables
    public enum TeleStates {
        START_STATE
    }

    private TeleStates currentState = TeleStates.START_STATE;

    //Motors


    //Servos


    //Misc
    FtcDashboard dash = null;
    Telemetry telemetryA = null;

    //All Functions
    public void configureMotors() {

    }

    @Override
    public void init() {
        configureMotors();
        dash = FtcDashboard.getInstance();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }


    //State Machine
    @Override
    public void loop() {

    }
}

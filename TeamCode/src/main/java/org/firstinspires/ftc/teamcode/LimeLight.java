package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
@TeleOp
public class LimeLight extends OpMode {

    private Telemetry telemetryA;
    private int cameraPixelsWidth = 480;
    public static double alignGain = .001;

    //0 = blue, 1 = red, 2 = yellow
    public static int pipeline = 0;


    public double theta = 0;
    public double avgTheta = 0;
    public double estDepth = 0;
    public double sum = 0;
    public int i = 0;

    public static int outCount = 6;
    public static int inCount = 50;


    private Limelight3A limelight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    FtcDashboard dash = null;

    private void drivePower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private double depthPoly(double x) {
//        return (0.000163173 * Math.pow(x, 3)) + (0.00314303 * Math.pow(x, 2)) + (0.311249 * x) + 10.77176; //V1
//        return (0.000448391 * Math.pow(x, 3)) + (0.00609961 * Math.pow(x, 2)) + (0.247723 * x) + 10.84374; //V2
        return (0.0002377 * Math.pow(x, 3)) + (0.00551973 * Math.pow(x, 2)) + (0.276311 * x) + 9.7189;
    }

    private double estimatedDepth(int avgCount, int dataCount) {
        double sumTheta = 0;
        double sum = 0;
        for (int i = 0; i < avgCount; i++) {
            double innerSum = 0;
            double innerThetaSum = 0;
            for (int e = 0; e < dataCount; e++) {
                innerSum += depthPoly(theta);
                innerThetaSum += theta;
            }
            sum += (innerSum / dataCount);
            sumTheta += (innerThetaSum / dataCount);
        }
        avgTheta = sumTheta / avgCount;
        return sum / avgCount;
    }

    private void motorConfig() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void init() {
        motorConfig();
        dash = FtcDashboard.getInstance();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(pipeline);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.addData(">", "Robot Ready.  Press Play.");
        telemetryA.update();
    }

    boolean pastBPressed = false;
    boolean pastYPressed = false;

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y * .5;
        double strafe = gamepad1.left_stick_x * .5;
        double yaw = gamepad1.right_stick_x * .5;


        LLResult result = limelight.getLatestResult();

        if (!pastBPressed && gamepad1.b) {
            pastBPressed = true;
            pipeline++;
            if (pipeline > 2)
                pipeline = 0;
            limelight.pipelineSwitch(pipeline);
        } else if (gamepad1.b) {
            pastBPressed = true;
        } else {
            pastBPressed = false;
        }

        if (result != null) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
//                telemetryA.addData("LL Latency", captureLatency + targetingLatency);
//                telemetryA.addData("Parse Latency", parseLatency);
//                telemetryA.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            if (result.isValid()) {
                telemetryA.addData("tx", result.getTx());
                telemetryA.addData("txnc", result.getTxNC());
                telemetryA.addData("ty", result.getTy());
                telemetryA.addData("tync", result.getTyNC());

                theta = result.getTy();
                telemetryA.addData("Theta:", theta);
                telemetryA.addData("Depth in Inches:", depthPoly(theta));

                telemetryA.addData("Botpose", botpose.toString());

//                    // Access barcode results
//                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
//                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
//                        telemetryA.addData("Barcode", "Data: %s", br.getData());
//                    }
//
//                    // Access classifier results
//                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
//                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
//                        telemetryA.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
//                    }
//
//                    // Access detector results
//                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
//                    for (LLResultTypes.DetectorResult dr : detectorResults) {
//                        telemetryA.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
//                    }
//
//                    // Access fiducial results
//                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                        telemetryA.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                    }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                LLResultTypes.ColorResult biggest = colorResults.get(0);
//                    for (LLResultTypes.ColorResult cr : colorResults) {
//                        telemetryA.addData("Color", "X: %.2f, Y: %.2f, A: %.2f", cr.getTargetXPixels(), cr.getTargetYPixels(), cr.getTargetArea());
//                        if ((cr.getTargetArea() > biggest.getTargetArea()) && (cr.getTargetYPixels() > 350)) {
//                            biggest = cr;
//                        }
//                    }

                int error = (int) biggest.getTargetXPixels() - cameraPixelsWidth;
                double correctionPower = error * alignGain;

                telemetryA.addData("Power: ", correctionPower);

                if (gamepad1.right_bumper)
                    drivePower(correctionPower, -correctionPower, correctionPower, -correctionPower);

//                    if (biggest.getTargetXPixels() > 520) {
//                        drivePower(.35,-.35,-.35,.35);
//                    } else if (biggest.getTargetXPixels() < 440) {
//                        drivePower(-.35,.35,.35,-.35);
//                    } else {
//                        drivePower(0,0,0,0);
//                    }
            }
        } else {
            drivePower(0,0,0,0);
            telemetryA.addData("Limelight", "No data available");
        }


        if (!pastYPressed && gamepad1.y) {
            estDepth = estimatedDepth(outCount, inCount);
            pastYPressed = true;
        } else if (gamepad1.y) {
            pastYPressed = true;
        } else {
            pastYPressed = false;
        }

        telemetryA.addData("Estimated Depth:", estDepth);
        telemetryA.addData("Estimated Theta:", avgTheta);

        if (pipeline == 0)
            telemetryA.addLine("Pipleline: Blue");
        else if (pipeline == 1)
            telemetryA.addLine("Pipeline: Red");
        else if (pipeline == 2)
            telemetryA.addLine("Pipeline: Yellow");

        telemetryA.update();
    }

}

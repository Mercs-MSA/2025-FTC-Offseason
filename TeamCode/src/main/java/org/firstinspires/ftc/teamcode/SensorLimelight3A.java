/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@TeleOp
@Config
public class SensorLimelight3A extends LinearOpMode {

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
        //return (0.000163173 * Math.pow(x, 3)) + (0.00314303 * Math.pow(x, 2)) + (0.311249 * x) + 10.77176; //V1
        return (0.000448391 * Math.pow(x, 3)) + (0.00609961 * Math.pow(x, 2)) + (0.247723 * x) + 10.84374; //V2
    }

    private double estimatedDepth(int avgCount, int dataCount) {
        double sum = 0;
        for (int i = 0; i < avgCount; i++) {
            double innerSum = 0;
            for (int e = 0; e < dataCount; e++) {
                innerSum += depthPoly(theta);
            }
            sum += (innerSum / dataCount);
        }
        return sum / avgCount;
    }

//    private double estimatedDepth(int avgCount, int dataCount) {
//        double[] list = new double[dataCount];
//        double sum = 0;
//        for (int i = 0; i < avgCount; i++) {
//            double innerSum = 0;
//            for (int e = 0; e < dataCount; e++) {
//                list[e] = depthPoly(theta);
//            }
//            for (int o = 0; o < dataCount; o++) {
//
//            }
//            sum += (innerSum / dataCount);
//        }
//        return sum / avgCount;
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(pipeline);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.addData(">", "Robot Ready.  Press Play.");
        telemetryA.update();
        waitForStart();

        boolean pastBPressed = false;
        boolean pastYPressed = false;
        while (opModeIsActive()) {

            LimeLightImageTools ltool = new LimeLightImageTools(limelight);

//            LLStatus status = limelight.getStatus();
//            telemetryA.addData("Name", "%s",
//                    status.getName());
//            telemetryA.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                    status.getTemp(), status.getCpu(),(int)status.getFps());
//            telemetryA.addData("Pipeline", "Index: %d, Type: %s",
//                    status.getPipelineIndex(), status.getPipelineType());
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
                sleep(100);
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
//                    telemetryA.addData("Theta:", theta);
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

            i++;
            sum += depthPoly(theta);
//            sum += theta;
            if (!pastYPressed && gamepad1.y) {
//                estDepth = estimatedDepth(outCount, inCount);
                estDepth = sum/i;
                i = 0;
                sum = 0;
                pastYPressed = true;
            } else if (gamepad1.y) {
                pastYPressed = true;
            } else {
                pastYPressed = false;
            }

            telemetryA.addData("Estimated Depth:", estDepth);
//            telemetryA.addData("Estimated Theta:", avgTheta);

            if (pipeline == 0)
                telemetryA.addLine("Pipleline: Blue");
            else if (pipeline == 1)
                telemetryA.addLine("Pipeline: Red");
            else if (pipeline == 2)
                telemetryA.addLine("Pipeline: Yellow");

            telemetryA.update();
        }
        limelight.stop();
    }
}

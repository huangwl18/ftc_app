/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import android.graphics.Color;
import android.provider.CalendarContract;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="Autonomous_Red_JewelOnly", group="9367")
public class Autonomous_9367_Red_JewelOnly extends LinearOpMode {
    private VuforiaLocalizer vuforia;
    private PrivateData priv = new PrivateData();

    private DcMotor LFDrive, RFDrive, LRDrive, RRDrive, lifter1, lifter2;
    private Servo jewelArm, rearBumper1, rearBumper2;
    private ColorSensor jewelColorSensor;

    private String column;
    private double vuDetectionStartTime, initialHeading;

    double encoderFactor = 420 / 134.4;

    @Override
    public void runOpMode() throws InterruptedException {
        LFDrive = hardwareMap.get(DcMotor.class, "LFDrive");
        RFDrive = hardwareMap.get(DcMotor.class, "RFDrive");
        LRDrive = hardwareMap.get(DcMotor.class, "LRDrive");
        RRDrive = hardwareMap.get(DcMotor.class, "RRDrive");
        lifter1 = hardwareMap.get(DcMotor.class, "lifter1");
        lifter2 = hardwareMap.get(DcMotor.class, "lifter2");
        //relicArm = hardwareMap.get(DcMotor.class, "relicArm");

        jewelArm = hardwareMap.get(Servo.class, "jewelArm");
        rearBumper1 = hardwareMap.get(Servo.class, "rearBumper1");
        rearBumper2 = hardwareMap.get(Servo.class, "rearBumper2");
        //relicGrabber = hardwareMap.get(Servo.class, "relicGrabber");
        //relicLifter = hardwareMap.get(Servo.class, "relicLifter");

        jewelColorSensor = hardwareMap.get(ColorSensor.class, "jewelColorSensor");
        //lineColorSensor = hardwareMap.get(ColorSensor.class, "lineColorSensor");

        jewelArm.setPosition(0.8555);
        rearBumper1.setPosition(0.6828);
        rearBumper2.setPosition(0.2237);
        //relicGrabber.setPosition(0);
        //relicLifter.setPosition(0);
        RFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RRDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lifter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU_class imu = new IMU_class("imu", hardwareMap);

        while (!isStarted()) {
            telemetry.addData("Status", "Initialization Complete");
            telemetry.update();
        }

        // START VUFORIA

        //make camera view show up on screen
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = priv.vuforiaKey;


        //use back camera
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //Shows XYZ axes on detected object (Teapots, buildings, and none also valid)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.TEAPOT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Get image identification files
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        // END VUFORIA

        initialHeading = getHeading(imu);

        waitForStart();

        //get initial orientation

        //knock the jewel
        jewelArm.setPosition(0.2227);
        Thread.sleep(1000);
        boolean jewelDetected = false;
        double jewelDetectionStartTime = System.currentTimeMillis();
        while (!jewelDetected && (System.currentTimeMillis() - jewelDetectionStartTime) < 3000) {
            if (jewelColorSensor.red() > jewelColorSensor.blue() + 5) {
                jewelDetected = true;
                turn2Angle(12, imu, 1.3, false);
                Thread.sleep(100);
                jewelArm.setPosition(0.8555);
                Thread.sleep(500);
                turn2Angle(-12, imu, 1.3, false);
            } else if (jewelColorSensor.blue() > jewelColorSensor.red() + 5) {
                jewelDetected = true;
                turn2Angle(-12, imu, 1.3, false);
                Thread.sleep(100);
                jewelArm.setPosition(0.8555);
                Thread.sleep(500);
                turn2Angle(12, imu, 1.3, false);
            } else {
                continue;
            }
        }
        jewelArm.setPosition(0.8555);
        //end knocking the jewel
        Thread.sleep(200);


    }

    double getHeading(IMU_class a) {
        return a.getAngles()[0];
    }

    /* Turning method for IMU
       target > 0 -> Turn Left
       target < 0 -> Turn Right
    */
    void turn2Angle(double target, IMU_class i, double decayRate, boolean bypass) {
        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startHeading = getHeading(i);
        double relTarget = startHeading + target;
        double bypassCoefficient;
        if (0 > target) {
            //Turn right
            double difference = 180;
            while (difference > 2.5) {
                difference = Math.abs(relTarget - getHeading(i));
                if(difference > 30){
                    if(bypass){
                        LFDrive.setPower(.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) + 0.15 / 3);
                        LRDrive.setPower(.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) + 0.15 / 3);
                        RFDrive.setPower(-.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) - 0.15 / 3);
                        RRDrive.setPower(-.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) - 0.15 / 3);
                    }
                    else{
                        LFDrive.setPower(0.4 / 3);
                        LRDrive.setPower(0.4 / 3);
                        RFDrive.setPower(- 0.4 / 3);
                        RRDrive.setPower(- 0.4 / 3);
                    }
                }
                else{
                    LFDrive.setPower(.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) + 0.15 / 3);
                    LRDrive.setPower(.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) + 0.15 / 3);
                    RFDrive.setPower(-.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) - 0.15 / 3);
                    RRDrive.setPower(-.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) - 0.15 / 3);
                }

                telemetry.addData("degrees to target", Math.abs(getHeading(i) - relTarget));
                telemetry.addData("current heading", getHeading(i));
                telemetry.update();
            }
        } else {
            //Turn left
            double difference = 180;
            while (Math.abs(relTarget - getHeading(i)) > 2.5) {
                difference = Math.abs(relTarget - getHeading(i));
                if(difference > 30){
                    if(bypass){
                        LFDrive.setPower(-.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) - 0.15 / 3);
                        LRDrive.setPower(-.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) - 0.15 / 3);
                        RFDrive.setPower(.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) + 0.15 / 3);
                        RRDrive.setPower(.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) + 0.15 / 3);
                    }
                    else{
                        LFDrive.setPower(-0.4 / 3);
                        LRDrive.setPower(-0.4 / 3);
                        RFDrive.setPower(0.4 / 3);
                        RRDrive.setPower(0.4 / 3);
                    }
                }
                else{
                    LFDrive.setPower(-.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) - 0.15 / 3);
                    LRDrive.setPower(-.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) - 0.15 / 3);
                    RFDrive.setPower(.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) + 0.15 / 3);
                    RRDrive.setPower(.65 / 3 * Math.pow(difference / Math.abs(target), decayRate) + 0.15 / 3);
                }

                telemetry.addData("degrees to target", Math.abs(getHeading(i) - relTarget));
                telemetry.addData("current heading", getHeading(i));
                telemetry.update();
            }
        }
        LFDrive.setPower(0);
        LRDrive.setPower(0);
        RFDrive.setPower(0);
        RRDrive.setPower(0);
    }

    void moveWithEncoder(double power, int distance, String direction) {
        LFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean LFisFinished = false;
        boolean LRisFinished = false;
        boolean RFisFinished = false;
        boolean RRisFinished = false;
        power = power / 3;
        distance = (int) (distance / encoderFactor);
        if (direction.equalsIgnoreCase("FORWARD")) {
            LFDrive.setTargetPosition(LFDrive.getCurrentPosition() + distance);
            LRDrive.setTargetPosition(LRDrive.getCurrentPosition() + distance);
            RFDrive.setTargetPosition(RFDrive.getCurrentPosition() + distance);
            RRDrive.setTargetPosition(RRDrive.getCurrentPosition() + distance);
            LFDrive.setPower(power);
            LRDrive.setPower(power);
            RFDrive.setPower(power);
            RRDrive.setPower(power);
            while (!LFisFinished || !LRisFinished || !RFisFinished || !RRisFinished) {
                LFisFinished = Math.abs(LFDrive.getCurrentPosition() - LFDrive.getTargetPosition()) < 15;
                LRisFinished = Math.abs(LRDrive.getCurrentPosition() - LRDrive.getTargetPosition()) < 15;
                RFisFinished = Math.abs(RFDrive.getCurrentPosition() - RFDrive.getTargetPosition()) < 15;
                RRisFinished = Math.abs(RRDrive.getCurrentPosition() - RRDrive.getTargetPosition()) < 15;
                telemetry.addData("LFDrive encoder value", LFDrive.getCurrentPosition());
                telemetry.addData("LRDrive encoder value", LRDrive.getCurrentPosition());
                telemetry.addData("RFDrive encoder value", RFDrive.getCurrentPosition());
                telemetry.addData("RRDrive encoder value", RRDrive.getCurrentPosition());
                telemetry.update();
            }
        } else if (direction.equalsIgnoreCase("BACKWARD")) {
            LFDrive.setTargetPosition(LFDrive.getCurrentPosition() - distance);
            LRDrive.setTargetPosition(LRDrive.getCurrentPosition() - distance);
            RFDrive.setTargetPosition(RFDrive.getCurrentPosition() - distance);
            RRDrive.setTargetPosition(RRDrive.getCurrentPosition() - distance);
            LFDrive.setPower(-power);
            LRDrive.setPower(-power);
            RFDrive.setPower(-power);
            RRDrive.setPower(-power);
            while (!LFisFinished || !LRisFinished || !RFisFinished || !RRisFinished) {
                LFisFinished = Math.abs(LFDrive.getCurrentPosition() - LFDrive.getTargetPosition()) < 15;
                LRisFinished = Math.abs(LRDrive.getCurrentPosition() - LRDrive.getTargetPosition()) < 15;
                RFisFinished = Math.abs(RFDrive.getCurrentPosition() - RFDrive.getTargetPosition()) < 15;
                RRisFinished = Math.abs(RRDrive.getCurrentPosition() - RRDrive.getTargetPosition()) < 15;
                telemetry.addData("LFDrive encoder value", LFDrive.getCurrentPosition());
                telemetry.addData("LRDrive encoder value", LRDrive.getCurrentPosition());
                telemetry.addData("RFDrive encoder value", RFDrive.getCurrentPosition());
                telemetry.addData("RRDrive encoder value", RRDrive.getCurrentPosition());
                telemetry.update();
            }
        } else if (direction.equalsIgnoreCase("LEFT")) {
            LFDrive.setTargetPosition(LFDrive.getCurrentPosition() - distance);
            LRDrive.setTargetPosition(LRDrive.getCurrentPosition() + distance);
            RFDrive.setTargetPosition(RFDrive.getCurrentPosition() + distance);
            RRDrive.setTargetPosition(RRDrive.getCurrentPosition() - distance);
            LFDrive.setPower(-power);
            LRDrive.setPower(power);
            RFDrive.setPower(power);
            RRDrive.setPower(-power);
            while (!LFisFinished || !LRisFinished || !RFisFinished || !RRisFinished) {
                LFisFinished = Math.abs(LFDrive.getCurrentPosition() - LFDrive.getTargetPosition()) < 15;
                LRisFinished = Math.abs(LRDrive.getCurrentPosition() - LRDrive.getTargetPosition()) < 15;
                RFisFinished = Math.abs(RFDrive.getCurrentPosition() - RFDrive.getTargetPosition()) < 15;
                RRisFinished = Math.abs(RRDrive.getCurrentPosition() - RRDrive.getTargetPosition()) < 15;
                telemetry.addData("LFDrive encoder value", LFDrive.getCurrentPosition());
                telemetry.addData("LRDrive encoder value", LRDrive.getCurrentPosition());
                telemetry.addData("RFDrive encoder value", RFDrive.getCurrentPosition());
                telemetry.addData("RRDrive encoder value", RRDrive.getCurrentPosition());
                telemetry.update();
            }
        } else if (direction.equalsIgnoreCase("RIGHT")) {
            LFDrive.setTargetPosition(LFDrive.getCurrentPosition() + distance);
            LRDrive.setTargetPosition(LRDrive.getCurrentPosition() - distance);
            RFDrive.setTargetPosition(RFDrive.getCurrentPosition() - distance);
            RRDrive.setTargetPosition(RRDrive.getCurrentPosition() + distance);
            LFDrive.setPower(power);
            LRDrive.setPower(-power);
            RFDrive.setPower(-power);
            RRDrive.setPower(power);
            while (!LFisFinished || !LRisFinished || !RFisFinished || !RRisFinished) {
                LFisFinished = Math.abs(LFDrive.getCurrentPosition() - LFDrive.getTargetPosition()) < 15;
                LRisFinished = Math.abs(LRDrive.getCurrentPosition() - LRDrive.getTargetPosition()) < 15;
                RFisFinished = Math.abs(RFDrive.getCurrentPosition() - RFDrive.getTargetPosition()) < 15;
                RRisFinished = Math.abs(RRDrive.getCurrentPosition() - RRDrive.getTargetPosition()) < 15;
                telemetry.addData("LFDrive encoder value", LFDrive.getCurrentPosition());
                telemetry.addData("LRDrive encoder value", LRDrive.getCurrentPosition());
                telemetry.addData("RFDrive encoder value", RFDrive.getCurrentPosition());
                telemetry.addData("RRDrive encoder value", RRDrive.getCurrentPosition());
                telemetry.update();
            }
        } else {
            telemetry.addLine("Error: invalid input for direction");
        }
        LFDrive.setPower(0);
        LRDrive.setPower(0);
        RFDrive.setPower(0);
        RRDrive.setPower(0);
        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

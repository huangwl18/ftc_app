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
import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="Autonomous_Blue_Vertical", group="9367")
public class Autonomous_9367_Blue_Position2 extends LinearOpMode {
    private VuforiaLocalizer vuforia;
    private PrivateData priv = new PrivateData();

    private DcMotor LFDrive, RFDrive, LRDrive, RRDrive, lifter1, lifter2;
    private Servo jewelArm;
    private CRServo intakeTopLeft, intakeTopRight, intakeDownLeft, intakeDownRight;
    private ColorSensor jewelColorSensor, lineColorSensor;

    private String column;
    private double vuDetectionStartTime, initialHeading;

    double encoderFactor = 420 / 134.4;
    double jewelArmUp = 0.5;
    double jewelArmDown = 0.5;
    double jewelArmDown_adjust = 0.5;

    @Override
    public void runOpMode() throws InterruptedException{
        LFDrive = hardwareMap.get(DcMotor.class, "LFDrive");
        RFDrive = hardwareMap.get(DcMotor.class, "RFDrive");
        LRDrive = hardwareMap.get(DcMotor.class, "LRDrive");
        RRDrive = hardwareMap.get(DcMotor.class, "RRDrive");
        lifter1 = hardwareMap.get(DcMotor.class, "lifter1");
        lifter2 = hardwareMap.get(DcMotor.class, "lifter2");

        jewelArm = hardwareMap.get(Servo.class, "jewelArm");

        jewelColorSensor = hardwareMap.get(ColorSensor.class, "jewelColorSensor");
        lineColorSensor = hardwareMap.get(ColorSensor.class, "lineColorSensor");

        intakeTopLeft = hardwareMap.get(CRServo.class, "intakeTopLeft");
        intakeTopRight = hardwareMap.get(CRServo.class, "intakeTopRight");
        intakeDownLeft = hardwareMap.get(CRServo.class, "intakeDownLeft");
        intakeDownRight = hardwareMap.get(CRServo.class, "intakeDownRight");

        jewelArm.setPosition(jewelArmUp);
        RFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RRDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lifter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter1.setPower(0);
        lifter2.setPower(0);

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


        //use front camera
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        //Shows XYZ axes on detected object (Teapots, buildings, and none also valid)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.TEAPOT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Get image identification files
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        // END VUFORIA

        //get initial orientation
        initialHeading = getHeading(imu);

        waitForStart();

        lifter1.setPower(-.7);
        lifter2.setPower(-.7);
        //knock the jewel
        jewelArm.setPosition(jewelArmDown);
        Thread.sleep(500);
        lifter1.setPower(0);
        lifter2.setPower(0);
        Thread.sleep(1300);
        boolean jewelDetected = false;
        double jewelDetectionStartTime = System.currentTimeMillis();
        while (opModeIsActive() && !jewelDetected && (System.currentTimeMillis() - jewelDetectionStartTime) < 3000) {
            if((System.currentTimeMillis() - jewelDetectionStartTime) > 1000){
                jewelArm.setPosition(jewelArmDown_adjust);
            }
            if (jewelColorSensor.red() > jewelColorSensor.blue() + 10) {
                jewelDetected = true;
                turn2Angle(-12, imu, 1.3);
                Thread.sleep(100);
                jewelArm.setPosition(jewelArmUp);
                Thread.sleep(500);
                turn2Angle(12, imu, 1.3);
            } else if (jewelColorSensor.blue() > jewelColorSensor.red() + 10) {
                jewelDetected = true;
                turn2Angle(12, imu, 1.3);
                Thread.sleep(100);
                jewelArm.setPosition(jewelArmUp);
                Thread.sleep(500);
                turn2Angle(-12, imu, 1.3);
            }
            telemetry.addData("jewelColorSensor red: ", jewelColorSensor.red());
            telemetry.addData("jewelColorSensor green: ", jewelColorSensor.green());
            telemetry.addData("jewelColorSensor blue: ", jewelColorSensor.blue());
            telemetry.update();
        }
        jewelArm.setPosition(jewelArmUp);
        //end knocking the jewel
        Thread.sleep(200);

        //start Vuforia search
        relicTrackables.activate();
        //Thread.sleep(1000);
        vuDetectionStartTime = System.currentTimeMillis();
        turn2Angle(22.5, imu, 1.8);
        Thread.sleep(200);
        while(opModeIsActive()){
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("Key: ", vuMark);
                telemetry.update();
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    column = "RIGHT";
                    break;
                }
                else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    column = "CENTER";
                    break;
                }
                else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    column = "LEFT";
                    break;
                }

            }
            else{
                telemetry.addData("No key detected!", null);
                if(System.currentTimeMillis() - vuDetectionStartTime > 2500){
                    column = "UNKNOWN";
                    break;
                }
            }
            telemetry.update();
        }
        relicTrackables.deactivate();
        turn2Angle(-22.5, imu, 1.8);
        // End Vuforia search

        //move down the balancing stone
        moveWithEncoder(1, 3600, "Backward");

        turn2Angle(initialHeading - 180, imu, 1.8);

        //move away from blue line
        moveWithEncoder(0.9, 4500, "Right");
        //Thread.sleep(500);

        //Search blue line and move to the center
        SearchBlueLine();
        Thread.sleep(500);
        telemetry.addLine("Search complete");

        //condition 1: left column
        if(column.equalsIgnoreCase("Left")){
            //adjust to the left column
            moveWithEncoder(.9, 1050, "Left");
            //move toward cryptobox
            moveWithEncoder(.9, 720, "Forward");
            //release the glyph
            intakeDownLeft.setPower(-.7);
            intakeDownRight.setPower(.7);
            Thread.sleep(1200);
            intakeDownLeft.setPower(0);
            intakeDownRight.setPower(0);
            //back up a bit so that the robot does not touch the glyph
            moveWithEncoder(.9, 600, "Backward");
            //lift up so that the robot does not touch the glyph
            lifter1.setPower(-.5);
            lifter2.setPower(-.5);
            Thread.sleep(600);
            lifter1.setPower(0);
            lifter2.setPower(0);
            //move back to the center
            //moveWithEncoder(.9, 1280, "Right");
        }
        //condition 2: right column
        else if(column.equalsIgnoreCase("Right")){
            //adjust to the right column
            moveWithEncoder(.9, 1050, "Right");
            //move toward cryptobox
            moveWithEncoder(.9, 720, "Forward");
            //release the glyph
            intakeDownLeft.setPower(-.7);
            intakeDownRight.setPower(.7);
            Thread.sleep(1200);
            intakeDownLeft.setPower(0);
            intakeDownRight.setPower(0);
            //back up a bit so that the robot does not touch the glyph
            moveWithEncoder(.9, 680, "Backward");
            //lift up so that the robot does not touch the glyph
            lifter1.setPower(-.5);
            lifter2.setPower(-.5);
            Thread.sleep(600);
            lifter1.setPower(0);
            lifter2.setPower(0);
            //move back to the center
            //moveWithEncoder(.9, 1280, "Left");
        }
        //condition 3: center column or undetected pictograph
        else{
            //move toward cryptobox
            moveWithEncoder(.9, 720, "Forward");
            //release the glyph
            intakeDownLeft.setPower(-.7);
            intakeDownRight.setPower(.7);
            Thread.sleep(1200);
            intakeDownLeft.setPower(0);
            intakeDownRight.setPower(0);
            //back up a bit so that the robot does not touch the glyph
            moveWithEncoder(.9, 600, "Backward");
            //lift up so that the robot does not touch the glyph
            lifter1.setPower(-.5);
            lifter2.setPower(-.5);
            Thread.sleep(600);
            lifter1.setPower(0);
            lifter2.setPower(0);
        }
        Thread.sleep(200);







    }

    double getHeading(IMU_class a){
        return a.getAngles()[0];
    }

    /* Turning method for IMU
       target > 0 -> Turn Left
       target < 0 -> Turn Right
    */
    void turn2Angle(double target, IMU_class i, double decayRate) {
        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startHeading = getHeading(i);
        double relTarget = startHeading + target;
        if (0 > target) {
            //Turn right
            double difference;
            float startTime = System.currentTimeMillis();
            while (opModeIsActive() && Math.abs(relTarget - getHeading(i)) > 2.5 && (System.currentTimeMillis() - startTime) < 5000) {
                difference = Math.abs(relTarget - getHeading(i));
                if(difference > 30){
                    LFDrive.setPower(0.9 / 3);
                    LRDrive.setPower(0.9 / 3);
                    RFDrive.setPower(- 0.9 / 3);
                    RRDrive.setPower(- 0.9 / 3);
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
            double difference;
            float startTime = System.currentTimeMillis();
            while (opModeIsActive() && Math.abs(relTarget - getHeading(i)) > 2.5 && (System.currentTimeMillis() - startTime) < 5000) {
                difference = Math.abs(relTarget - getHeading(i));
                if(difference > 30){
                    LFDrive.setPower(-0.9 / 3);
                    LRDrive.setPower(-0.9 / 3);
                    RFDrive.setPower(0.9 / 3);
                    RRDrive.setPower(0.9 / 3);
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

    void moveWithEncoder(double power, int distance, String direction){
        LFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean LFisFinished = false;
        boolean LRisFinished = false;
        boolean RFisFinished = false;
        boolean RRisFinished = false;
        power = power/3;
        distance = (int)(distance/encoderFactor);
        if(direction.equalsIgnoreCase("FORWARD")){
            LFDrive.setTargetPosition(LFDrive.getCurrentPosition() + distance);
            LRDrive.setTargetPosition(LRDrive.getCurrentPosition() + distance);
            RFDrive.setTargetPosition(RFDrive.getCurrentPosition() + distance);
            RRDrive.setTargetPosition(RRDrive.getCurrentPosition() + distance);
            LFDrive.setPower(power);
            LRDrive.setPower(power);
            RFDrive.setPower(power);
            RRDrive.setPower(power);
            while(!LFisFinished || !LRisFinished || !RFisFinished || !RRisFinished){
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
        }
        else if(direction.equalsIgnoreCase("BACKWARD")){
            LFDrive.setTargetPosition(LFDrive.getCurrentPosition() - distance);
            LRDrive.setTargetPosition(LRDrive.getCurrentPosition() - distance);
            RFDrive.setTargetPosition(RFDrive.getCurrentPosition() - distance);
            RRDrive.setTargetPosition(RRDrive.getCurrentPosition() - distance);
            LFDrive.setPower(-power);
            LRDrive.setPower(-power);
            RFDrive.setPower(-power);
            RRDrive.setPower(-power);
            while(!LFisFinished || !LRisFinished || !RFisFinished || !RRisFinished){
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
        }
        else if(direction.equalsIgnoreCase("LEFT")){
            LFDrive.setTargetPosition(LFDrive.getCurrentPosition() - distance);
            LRDrive.setTargetPosition(LRDrive.getCurrentPosition() + distance);
            RFDrive.setTargetPosition(RFDrive.getCurrentPosition() + distance);
            RRDrive.setTargetPosition(RRDrive.getCurrentPosition() - distance);
            LFDrive.setPower(-power);
            LRDrive.setPower(power);
            RFDrive.setPower(power);
            RRDrive.setPower(-power);
            while(!LFisFinished || !LRisFinished || !RFisFinished || !RRisFinished){
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
        }
        else if(direction.equalsIgnoreCase("RIGHT")){
            LFDrive.setTargetPosition(LFDrive.getCurrentPosition() + distance);
            LRDrive.setTargetPosition(LRDrive.getCurrentPosition() - distance);
            RFDrive.setTargetPosition(RFDrive.getCurrentPosition() - distance);
            RRDrive.setTargetPosition(RRDrive.getCurrentPosition() + distance);
            LFDrive.setPower(power);
            LRDrive.setPower(-power);
            RFDrive.setPower(-power);
            RRDrive.setPower(power);
            while(!LFisFinished || !LRisFinished || !RFisFinished || !RRisFinished){
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
        }
        else{
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

    void SearchBlueLine(){
        //set mode
        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set boolean variables to detect the transitions
        boolean blueLineDetected1 = false;
        boolean greyMatDetected1 = false;
        boolean blueLineDetected2 = false;
        boolean greyMatDetected2 = false;
        //start moving left
        LFDrive.setPower(-.4/3);
        LRDrive.setPower(.4/3);
        RFDrive.setPower(.4/3);
        RRDrive.setPower(-.4/3);
        //set timeout variable
        long startTime = System.currentTimeMillis();
        //detect the first transition from mat to red line
        while(!blueLineDetected1 && (System.currentTimeMillis() - startTime) < 5000){
            blueLineDetected1 = (lineColorSensor.blue() - (lineColorSensor.red() + lineColorSensor.green()) / 2) > 10;
            if(blueLineDetected1){
                telemetry.addLine("first blue line detected");
            }
        }
        //record start position
        int LFStartEncoderValue = LFDrive.getCurrentPosition();
        int LRStartEncoderValue = LRDrive.getCurrentPosition();
        int RFStartEncoderValue = RFDrive.getCurrentPosition();
        int RRStartEncoderValue = RRDrive.getCurrentPosition();
        //set timeout variable
        startTime = System.currentTimeMillis();
        //detect the first transition from red line to mat
        while(!greyMatDetected1 && (System.currentTimeMillis() - startTime) < 2000){
            greyMatDetected1 = Math.abs(lineColorSensor.blue() - (lineColorSensor.red() + lineColorSensor.green()) / 2) < 10;
            if(greyMatDetected1){
                telemetry.addLine("grey mat detected");
            }
        }
        //set timeout variable
        startTime = System.currentTimeMillis();
        //detect the second transition from mat to red line
        while(!blueLineDetected2 && (System.currentTimeMillis() - startTime) < 7000){
            blueLineDetected2 = (lineColorSensor.blue() - (lineColorSensor.red() + lineColorSensor.green()) / 2) > 10;
            if(blueLineDetected2){
                telemetry.addLine("second blue line detected");
            }
        }
        //set timeout variable
        startTime = System.currentTimeMillis();
        //detect the second transition from red line to mat
        while(!greyMatDetected2 && (System.currentTimeMillis() - startTime) < 2000){
            greyMatDetected2 = Math.abs(lineColorSensor.blue() - (lineColorSensor.red() + lineColorSensor.green()) / 2) < 10;
            if(greyMatDetected2){
                telemetry.addLine("grey mat detected");
            }
        }
        //stop moving
        LFDrive.setPower(0);
        LRDrive.setPower(0);
        RFDrive.setPower(0);
        RRDrive.setPower(0);
        //calculate average distance travelled from the first transition to the last transition
        int LFDistanceTravelled = LFDrive.getCurrentPosition() - LFStartEncoderValue;
        int LRDistanceTravelled = LRDrive.getCurrentPosition() - LRStartEncoderValue;
        int RFDistanceTravelled = RFDrive.getCurrentPosition() - RFStartEncoderValue;
        int RRDistanceTravelled = RRDrive.getCurrentPosition() - RRStartEncoderValue;
        int avgDistanceTravelled = (Math.abs(LFDistanceTravelled) +
                Math.abs(LRDistanceTravelled) +
                Math.abs(RFDistanceTravelled) +
                Math.abs(RRDistanceTravelled)) / 4;
        //move right to the center column
        moveWithEncoder(.8, avgDistanceTravelled / 2 - 100, "Right");
    }
}

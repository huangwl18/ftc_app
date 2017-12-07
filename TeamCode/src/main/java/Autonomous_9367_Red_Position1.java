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

@Autonomous(name="Autonomous_Red_Position1", group="9367")
public class Autonomous_9367_Red_Position1 extends LinearOpMode {
    private VuforiaLocalizer vuforia;
    private PrivateData priv = new PrivateData();

    private DcMotor LFDrive, RFDrive, LRDrive, RRDrive, lifter1, lifter2;
    private Servo jewelArm, grabberL, grabberR, rearBumper1, rearBumper2;
    private ColorSensor jewelColorSensor, lineColorSensor;

    private String column;
    private double vuDetectionStartTime, initialHeading;

    @Override
    public void runOpMode() throws InterruptedException{
        LFDrive  = hardwareMap.get(DcMotor.class, "LFDrive");
        RFDrive = hardwareMap.get(DcMotor.class, "RFDrive");
        LRDrive  = hardwareMap.get(DcMotor.class, "LRDrive");
        RRDrive = hardwareMap.get(DcMotor.class, "RRDrive");
        lifter1 = hardwareMap.get(DcMotor.class, "lifter1");
        lifter2 = hardwareMap.get(DcMotor.class, "lifter2");
        //relicArm = hardwareMap.get(DcMotor.class, "relicArm");

        jewelArm = hardwareMap.get(Servo.class, "jewelArm");
        grabberL = hardwareMap.get(Servo.class, "grabberL");
        grabberR = hardwareMap.get(Servo.class, "grabberR");
        rearBumper1 = hardwareMap.get(Servo.class, "rearBumper1");
        rearBumper2 = hardwareMap.get(Servo.class, "rearBumper2");
        //relicGrabber = hardwareMap.get(Servo.class, "relicGrabber");
        //relicLifter = hardwareMap.get(Servo.class, "relicLifter");

        jewelColorSensor = hardwareMap.get(ColorSensor.class, "jewelColorSensor");
        lineColorSensor = hardwareMap.get(ColorSensor.class, "lineColorSensor");

        jewelArm.setPosition(0.22);
        grabberL.setPosition(0.0594);
        grabberR.setPosition(0.98);
        rearBumper1.setPosition(0.9655);
        rearBumper2.setPosition(0.0155);
        //relicGrabber.setPosition(0);
        //relicLifter.setPosition(0);
        LFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LRDrive.setDirection(DcMotorSimple.Direction.REVERSE);

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

        waitForStart();

        //get initial orientation
        initialHeading = getHeading(imu);

        //lift up so that the robot does not touch the glyph
        lifter1.setPower(-.5);
        lifter2.setPower(-.5);
        Thread.sleep(1000);
        lifter1.setPower(0);
        lifter2.setPower(0);
        //open the grabber
        grabberL.setPosition(0.8);
        grabberR.setPosition(0.2);
        Thread.sleep(2000);
        //lift goes down to grab the glyph
        lifter1.setPower(.15);
        lifter2.setPower(.15);
        Thread.sleep(1200);
        lifter1.setPower(0);
        lifter2.setPower(0);
        //grab the glyph
        grabberL.setPosition(0.25);
        grabberR.setPosition(0.767);
        Thread.sleep(2000);
        //lift goes up
        lifter1.setPower(-.5);
        lifter2.setPower(-.5);
        Thread.sleep(1000);
        //stop the lift
        lifter1.setPower(0);
        lifter2.setPower(0);

        /*//knock the jewel
        jewelArm.setPosition(0.95);
        Thread.sleep(1000);
        boolean jewelDetected = false;
        double jewelDetectionStartTime = System.currentTimeMillis();
        while(!jewelDetected && (System.currentTimeMillis() - jewelDetectionStartTime) < 3000){
            if(jewelColorSensor.red() > jewelColorSensor.blue() + 20){
                jewelDetected = true;
                turn2Angle(20, imu, 0.9);
                Thread.sleep(1000);
                jewelArm.setPosition(0.369);
                Thread.sleep(800);
                turn2Angle(-20, imu, 0.9);
            }
            else if(jewelColorSensor.blue() > jewelColorSensor.red() + 20){
                jewelDetected = true;
                turn2Angle(-20, imu, 0.9);
                Thread.sleep(1000);
                jewelArm.setPosition(0.369);
                Thread.sleep(800);
                turn2Angle(20, imu, 0.9);
            }
            else{
                continue;
            }
        }
        jewelArm.setPosition(0.369);
         //end knocking the jewel*/
        Thread.sleep(500);
        // Start Vuforia object search
        relicTrackables.activate();
        //Thread.sleep(1000);
        vuDetectionStartTime = System.currentTimeMillis();
        turn2Angle(15, imu, 2);
        Thread.sleep(500);
        while(opModeIsActive()){
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("Key: ", vuMark);
                telemetry.update();
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    column = "RIGHT";
                    turn2Angle(-15, imu, 2);
                    break;
                }
                else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    column = "CENTER";
                    turn2Angle(-15, imu, 2);
                    break;
                }
                else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    column = "LEFT";
                    turn2Angle(-15, imu, 2);
                    break;
                }

            }

            else{
                telemetry.addData("No key detected!", null);
                if(System.currentTimeMillis() - vuDetectionStartTime > 5000){
                    column = "UNKNOWN";
                    turn2Angle(-15, imu, 2);
                    break;
                }
            }
            telemetry.update();
        }
        relicTrackables.deactivate();
        telemetry.addLine("Vuforia Search complete");
        Thread.sleep(1000); // End Vuforia search

        //move down the balancing stone
        moveWithEncoder(.6, 1500, "Backward");

        //adjust heading so that the robot faces the wall
        turn2Angle(initialHeading - getHeading(imu) + 90, imu, 1.5);
        Thread.sleep(1000);

        //move toward the balancing stone to further adjust heading
        moveWithEncoder(.5, 700, "Right");
        Thread.sleep(1000);

        //Search red line and move to the center
        SearchRedLine();
        Thread.sleep(1000);

        //condition 1: left column
        if(column.equalsIgnoreCase("Left")){
            //adjust to the left column
            moveWithEncoder(.4, 1360, "Left");
            //move toward cryptobox
            moveWithEncoder(.4, 680, "Forward");
            //release the glyph
            grabberL.setPosition(0.6);
            grabberR.setPosition(0.4);
            //back up a bit so that the robot does not touch the glyph
            moveWithEncoder(.4, 200, "Backward");
            //move back to the center
            moveWithEncoder(.4, 1360, "Right");
        }
        //condition 2: right column
        else if(column.equalsIgnoreCase("Right")){
            //adjust to the right column
            moveWithEncoder(.4, 1360, "Right");
            //move toward cryptobox
            moveWithEncoder(.4, 680, "Forward");
            //release the glyph
            grabberL.setPosition(0.6);
            grabberR.setPosition(0.4);
            //back up a bit so that the robot does not touch the glyph
            moveWithEncoder(.4, 200, "Backward");
            //move back to the center
            moveWithEncoder(.4, 1360, "Left");
        }
        //condition 3: center column or undetected pictograph
        else{
            //move toward cryptobox
            moveWithEncoder(.4, 680, "Forward");
            //release the glyph
            grabberL.setPosition(0.6);
            grabberR.setPosition(0.4);
            //back up a bit so that the robot does not touch the glyph
            moveWithEncoder(.4, 200, "Backward");
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
    void turn2Angle(double target, IMU_class i, double decayRate){
        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startHeading = getHeading(i);
        double relTarget = startHeading + target;
        if (0 > target){
            //Turn right
            double difference = 180;
            while (difference > 1){
                difference = Math.abs(relTarget - getHeading(i));
                LFDrive.setPower(.65 * Math.pow(difference / Math.abs(target), decayRate) + 0.15);
                LRDrive.setPower(.65 * Math.pow(difference / Math.abs(target), decayRate) + 0.15);
                RFDrive.setPower(-.65 * Math.pow(difference / Math.abs(target), decayRate) - 0.15);
                RRDrive.setPower(-.65 * Math.pow(difference / Math.abs(target), decayRate) - 0.15);
                telemetry.addData("degrees to target", Math.abs(getHeading(i) - relTarget));
                telemetry.addData("current heading", getHeading(i));
                telemetry.update();
                idle();
            }
        }
        else{
            //Turn left
            double difference = 180;
            while (Math.abs(relTarget - getHeading(i)) > 1){
                difference = Math.abs(relTarget - getHeading(i));
                LFDrive.setPower(-.65 * Math.pow(difference / Math.abs(target), decayRate) - 0.15);
                LRDrive.setPower(-.65 * Math.pow(difference / Math.abs(target), decayRate) - 0.15);
                RFDrive.setPower(.65 * Math.pow(difference / Math.abs(target), decayRate) + 0.15);
                RRDrive.setPower(.65 * Math.pow(difference / Math.abs(target), decayRate) + 0.15);
                telemetry.addData("degrees to target", Math.abs(getHeading(i) - relTarget));
                telemetry.addData("current heading", getHeading(i));
                telemetry.update();
                idle();
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
        if(direction.equalsIgnoreCase("FORWARD")){
            LFDrive.setTargetPosition(LFDrive.getCurrentPosition() + distance);
            LRDrive.setTargetPosition(LRDrive.getCurrentPosition() + distance);
            RFDrive.setTargetPosition(RFDrive.getCurrentPosition() + distance);
            RRDrive.setTargetPosition(RRDrive.getCurrentPosition() + distance);
            LFDrive.setPower(power);
            LRDrive.setPower(power);
            RFDrive.setPower(power);
            RRDrive.setPower(power);
            while(LFDrive.isBusy() || LRDrive.isBusy() || RFDrive.isBusy() || RRDrive.isBusy()){
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
            while(LFDrive.isBusy() || LRDrive.isBusy() || RFDrive.isBusy() || RRDrive.isBusy()){
                telemetry.addData("LFDrive encoder value", LFDrive.getCurrentPosition());
                telemetry.addData("LRDrive encoder value", LRDrive.getCurrentPosition());
                telemetry.addData("RFDrive encoder value", RFDrive.getCurrentPosition());
                telemetry.addData("RRDrive encoder value", RRDrive.getCurrentPosition());
                telemetry.update();
            }
        }
        else if(direction.equalsIgnoreCase("LEFT")){
            LFDrive.setTargetPosition(LFDrive.getCurrentPosition() + distance);
            LRDrive.setTargetPosition(LRDrive.getCurrentPosition() - distance);
            RFDrive.setTargetPosition(RFDrive.getCurrentPosition() - distance);
            RRDrive.setTargetPosition(RRDrive.getCurrentPosition() + distance);
            LFDrive.setPower(power);
            LRDrive.setPower(-power);
            RFDrive.setPower(-power);
            RRDrive.setPower(power);
            while(LFDrive.isBusy() || LRDrive.isBusy() || RFDrive.isBusy() || RRDrive.isBusy()){
                telemetry.addData("LFDrive encoder value", LFDrive.getCurrentPosition());
                telemetry.addData("LRDrive encoder value", LRDrive.getCurrentPosition());
                telemetry.addData("RFDrive encoder value", RFDrive.getCurrentPosition());
                telemetry.addData("RRDrive encoder value", RRDrive.getCurrentPosition());
                telemetry.update();
            }
        }
        else if(direction.equalsIgnoreCase("RIGHT")){
            LFDrive.setTargetPosition(LFDrive.getCurrentPosition() - distance);
            LRDrive.setTargetPosition(LRDrive.getCurrentPosition() + distance);
            RFDrive.setTargetPosition(RFDrive.getCurrentPosition() + distance);
            RRDrive.setTargetPosition(RRDrive.getCurrentPosition() - distance);
            LFDrive.setPower(-power);
            LRDrive.setPower(power);
            RFDrive.setPower(power);
            RRDrive.setPower(-power);
            while(LFDrive.isBusy() || LRDrive.isBusy() || RFDrive.isBusy() || RRDrive.isBusy()){
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

    void SearchRedLine(){
        //set mode
        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set boolean variables to detect the transitions
        boolean redLineDetected1 = false;
        boolean greyMatDetected1 = false;
        boolean redLineDetected2 = false;
        boolean greyMatDetected2 = false;
        //start moving left
        LFDrive.setPower(.3);
        LRDrive.setPower(-.3);
        RFDrive.setPower(-.3);
        RRDrive.setPower(.3);
        //set timeout variable
        long startTime = System.currentTimeMillis();
        //detect the first transition from mat to red line
        while(!redLineDetected1 && (System.currentTimeMillis() - startTime) < 5000){
            redLineDetected1 = (lineColorSensor.red() - (lineColorSensor.blue() + lineColorSensor.green()) / 2) > 8;
            if(redLineDetected1){
                telemetry.addLine("first red line detected");
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
            greyMatDetected1 = Math.abs(lineColorSensor.red() - (lineColorSensor.blue() + lineColorSensor.green()) / 2) < 3;
            if(greyMatDetected1){
                telemetry.addLine("grey mat detected");
            }
        }
        //set timeout variable
        startTime = System.currentTimeMillis();
        //detect the second transition from mat to red line
        while(!redLineDetected2 && (System.currentTimeMillis() - startTime) < 7000){
            redLineDetected2 = (lineColorSensor.red() - (lineColorSensor.blue() + lineColorSensor.green()) / 2) > 8;
            if(redLineDetected2){
                telemetry.addLine("second red line detected");
            }
        }
        //set timeout variable
        startTime = System.currentTimeMillis();
        //detect the second transition from red line to mat
        while(!greyMatDetected2 && (System.currentTimeMillis() - startTime) < 2000){
            greyMatDetected2 = Math.abs(lineColorSensor.red() - (lineColorSensor.blue() + lineColorSensor.green()) / 2) < 3;
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
        moveWithEncoder(.4, avgDistanceTravelled / 2, "Right");
    }
}

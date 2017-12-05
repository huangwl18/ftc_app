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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@Autonomous(name="Autonomous_Red_Position1", group="9367")
public class Autonomous_9367_Red_Position1 extends LinearOpMode {

    private DcMotor LFDrive, RFDrive, LRDrive, RRDrive, lifter1, lifter2;
    private Servo jewelArm, grabberL, grabberR, rearBumper1, rearBumper2;
    private ColorSensor jewelColorSensor;

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

        jewelArm.setPosition(0.145);
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

        waitForStart();
        //should turn right
        turn2Angle(-90, imu);
    }

    double getHeading(IMU_class a){
        return a.getAngles()[0];
    }

    // Turning method for IMU
    void turn2Angle(double target, IMU_class i){
        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double initialHeading = getHeading(i);
        double relTarget = initialHeading + target;
        if (0 > target){
            //Turn right
            double difference = 180;
            while (difference > 2){
                difference = Math.abs(relTarget - getHeading(i));
                LFDrive.setPower(.65 * Math.pow(difference / Math.abs(target), .8) + 0.15);
                LRDrive.setPower(.65 * Math.pow(difference / Math.abs(target), .8) + 0.15);
                RFDrive.setPower(-.65 * Math.pow(difference / Math.abs(target), .8) - 0.15);
                RRDrive.setPower(-.65 * Math.pow(difference / Math.abs(target), .8) - 0.15);
                telemetry.addData("degrees to target", Math.abs(getHeading(i) - relTarget));
                telemetry.addData("current heading", getHeading(i));
                telemetry.update();
                idle();
            }
        }
        else{
            //Turn left
            double difference = 180;
            while (Math.abs(relTarget - getHeading(i)) > 2){
                LFDrive.setPower(-.65 * Math.pow(difference / Math.abs(target), .8) - 0.15);
                LRDrive.setPower(-.65 * Math.pow(difference / Math.abs(target), .8) - 0.15);
                RFDrive.setPower(.65 * Math.pow(difference / Math.abs(target), .8) + 0.15);
                RRDrive.setPower(.65 * Math.pow(difference / Math.abs(target), .8) + 0.15);
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
}

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TeleOp_0126_TwoDrivers", group="9367")
public class TeleOp_0126_TwoDrivers extends OpMode
{

    private DcMotor LFDrive, RFDrive, LRDrive, RRDrive, lifter1, lifter2, grabberMotor;
    private Servo jewelArm, rearBumper1, rearBumper2;
    private CRServo intakeTopLeft, intakeTopRight, intakeDownLeft, intakeDownRight;
    private ColorSensor jewelColorSensor;


    @Override
    public void init() {

        LFDrive  = hardwareMap.get(DcMotor.class, "LFDrive");
        RFDrive = hardwareMap.get(DcMotor.class, "RFDrive");
        LRDrive  = hardwareMap.get(DcMotor.class, "LRDrive");
        RRDrive = hardwareMap.get(DcMotor.class, "RRDrive");
        lifter1 = hardwareMap.get(DcMotor.class, "lifter1");
        lifter2 = hardwareMap.get(DcMotor.class, "lifter2");
        grabberMotor = hardwareMap.get(DcMotor.class, "grabberMotor");

        jewelArm = hardwareMap.get(Servo.class, "jewelArm");
        rearBumper1 = hardwareMap.get(Servo.class, "rearBumper1");
        rearBumper2 = hardwareMap.get(Servo.class, "rearBumper2");

        jewelColorSensor = hardwareMap.get(ColorSensor.class, "jewelColorSensor");

        intakeTopLeft = hardwareMap.get(CRServo.class, "intakeTopLeft");
        intakeTopRight = hardwareMap.get(CRServo.class, "intakeTopRight");
        intakeDownLeft = hardwareMap.get(CRServo.class, "intakeDownLeft");
        intakeDownRight = hardwareMap.get(CRServo.class, "intakeDownRight");

        jewelArm.setPosition(0.8555);
        rearBumper1.setPosition(0.9655);
        rearBumper2.setPosition(0.0155);
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
        grabberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //disable color sensors during TeleOp
        //jewelColorSensor.enableLed(false);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if(gamepad1.right_trigger > 0.5){
            LFDrive.setPower(0.2 * (-gamepad1.left_stick_y + gamepad1.left_stick_x + 0.6 * gamepad1.right_stick_x));
            LRDrive.setPower(0.2 * (-gamepad1.left_stick_y - gamepad1.left_stick_x + 0.6 * gamepad1.right_stick_x));
            RFDrive.setPower(0.2 * (-gamepad1.left_stick_y - gamepad1.left_stick_x - 0.6 * gamepad1.right_stick_x));
            RRDrive.setPower(0.2 * (-gamepad1.left_stick_y + gamepad1.left_stick_x - 0.6 * gamepad1.right_stick_x));
        }
        else if(gamepad1.left_trigger > 0.5){
            LFDrive.setPower(0.4 * (-gamepad1.left_stick_y + gamepad1.left_stick_x + 0.6 * gamepad1.right_stick_x));
            LRDrive.setPower(0.4 * (-gamepad1.left_stick_y - gamepad1.left_stick_x + 0.6 * gamepad1.right_stick_x));
            RFDrive.setPower(0.4 * (-gamepad1.left_stick_y - gamepad1.left_stick_x - 0.6 * gamepad1.right_stick_x));
            RRDrive.setPower(0.4 * (-gamepad1.left_stick_y + gamepad1.left_stick_x - 0.6 * gamepad1.right_stick_x));
        }
        else{
            LFDrive.setPower(.6 * (-gamepad1.left_stick_y + gamepad1.left_stick_x + 0.6 * gamepad1.right_stick_x));
            LRDrive.setPower(.6 * (-gamepad1.left_stick_y - gamepad1.left_stick_x + 0.6 * gamepad1.right_stick_x));
            RFDrive.setPower(.6 * (-gamepad1.left_stick_y - gamepad1.left_stick_x - 0.6 * gamepad1.right_stick_x));
            RRDrive.setPower(.6 * (-gamepad1.left_stick_y + gamepad1.left_stick_x - 0.6 * gamepad1.right_stick_x));
        }



        if(gamepad2.right_trigger > 0.5){
            lifter1.setPower(-.7);
            lifter2.setPower(-.7);
        }
        else if(gamepad2.left_trigger > 0.5){
            lifter1.setPower(0.15);
            lifter2.setPower(0.15);
        }
        else{
            lifter1.setPower(0);
            lifter2.setPower(0);
        }

        /*
        if(gamepad2.y){
            rearBumper1.setPosition(0.9655);
            rearBumper2.setPosition(0.0155);
        }
        else if(gamepad2.x){
            rearBumper1.setPosition(0);
            rearBumper2.setPosition(1);
        }*/

        if(gamepad2.right_stick_y > 0.5){
            intakeDownLeft.setPower(.7);
            intakeDownRight.setPower(-.7);
        }
        else if(gamepad2.right_stick_y < -0.5){
            intakeDownLeft.setPower(-.7);
            intakeDownRight.setPower(.7);
        }
        else{
            intakeDownLeft.setPower(0);
            intakeDownRight.setPower(0);
        }

        if(gamepad2.left_stick_y > 0.5){
            intakeTopLeft.setPower(-.7);
            intakeTopRight.setPower(.7);
        }
        else if(gamepad2.left_stick_y < -0.5){
            intakeTopLeft.setPower(.7);
            intakeTopRight.setPower(-.7);
        }
        else{
            intakeTopLeft.setPower(0);
            intakeTopRight.setPower(0);
        }

        if(gamepad2.dpad_right && gamepad2.y){
            grabberMotor.setPower(1);
        }
        else if(gamepad2.dpad_left && gamepad2.y){
            grabberMotor.setPower(-1);
        }
        else if(gamepad2.dpad_right && !gamepad2.y){
            grabberMotor.setPower(0.5);
        }
        else if(gamepad2.dpad_left && !gamepad2.y){
            grabberMotor.setPower(-0.5);
        }
        else{
            grabberMotor.setPower(0);
        }

        if(gamepad2.x){
            rearBumper1.setPosition(0.0188);
            rearBumper2.setPosition(0.8877);
        }
        else{
            rearBumper1.setPosition(0.6828);
            rearBumper2.setPosition(0.2237);
        }


        telemetry.addData("jewelArm Position: ", jewelArm.getPosition());
        telemetry.addData("jewelColorSensor red: ", jewelColorSensor.red());
        telemetry.addData("jewelColorSensor green: ", jewelColorSensor.green());
        telemetry.addData("jewelColorSensor blue: ", jewelColorSensor.blue());
        telemetry.addData("intakeTopLeft: ", intakeTopLeft.getPower());
        telemetry.addData("intakeTopRight: ", intakeTopRight.getPower());
        telemetry.addData("intakeDownLeft: ", intakeDownLeft.getPower());
        telemetry.addData("intakeDownRight: ", intakeDownRight.getPower());

        telemetry.addData("rearBumper1: ", rearBumper1.getPosition());
        telemetry.addData("rearBumper2: ", rearBumper2.getPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

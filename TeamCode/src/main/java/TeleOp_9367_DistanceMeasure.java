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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TeleOp_9367_DistanceMeasure", group="9367")
public class TeleOp_9367_DistanceMeasure extends OpMode
{

    private DcMotor LFDrive, RFDrive, LRDrive, RRDrive, lifter1, lifter2;
    private Servo jewelArm, grabberL, grabberR, rearBumper1, rearBumper2;
    private ColorSensor jewelColorSensor, lineColorSensor;

    @Override
    public void init() {

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

        jewelArm.setPosition(0.258);
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

        if(gamepad1.left_bumper){
            LFDrive.setPower(0.3 * (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            LRDrive.setPower(0.3 * (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            RFDrive.setPower(0.3 * (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            RRDrive.setPower(0.3 * (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
        }
       /* else if(gamepad1.left_trigger > 0.5){
            LFDrive.setPower(0.4 * (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            LRDrive.setPower(0.4 * (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            RFDrive.setPower(0.4 * (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            RRDrive.setPower(0.4 * (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
        }*/
        else{
            LFDrive.setPower(0.5 * (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            LRDrive.setPower(0.5 * (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            RFDrive.setPower(0.5 * (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            RRDrive.setPower(0.5 * (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
        }


        /*if(gamepad1.right_bumper){
            grabberL.setPosition(0.25);
            grabberR.setPosition(0.767);
        }
        else{
            grabberL.setPosition(0.6);
            grabberR.setPosition(0.4);
        }*/


        if(gamepad1.a){
            jewelArm.setPosition(0.84);
        }
        else{
            jewelArm.setPosition(0.258);
        }

        if(gamepad1.right_trigger > 0.5){
            lifter1.setPower(-0.75);
            lifter2.setPower(-0.75);
        }
        else if(gamepad1.left_trigger > 0.5){
            lifter1.setPower(0.15);
            lifter2.setPower(0.15);
        }
        else{
            lifter1.setPower(0);
            lifter2.setPower(0);
        }

        if(gamepad1.y){
            rearBumper1.setPosition(0.9655);
            rearBumper2.setPosition(0.0155);
        }
        else if(gamepad1.x){
            rearBumper1.setPosition(0);
            rearBumper2.setPosition(1);
        }

        if(gamepad1.dpad_up){
            LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(gamepad2.dpad_up){
            grabberL.setPosition(grabberL.getPosition() + 0.005);
        }
        if(gamepad2.dpad_down){
            grabberL.setPosition(grabberL.getPosition() - 0.005);
        }
        if(gamepad2.dpad_left){
            grabberR.setPosition(grabberR.getPosition() + 0.005);
        }
        if(gamepad2.dpad_right){
            grabberR.setPosition(grabberR.getPosition() - 0.005);
        }

        telemetry.addData("jewelColor_blueValue ", jewelColorSensor.blue());
        telemetry.addData("jewelColor_redValue ", jewelColorSensor.red());
        telemetry.addData("LFDrive_encoder", LFDrive.getCurrentPosition());
        telemetry.addData("LRDrive_encoder", LRDrive.getCurrentPosition());
        telemetry.addData("RFDrive_encoder", RFDrive.getCurrentPosition());
        telemetry.addData("RRDrive_encoder", RRDrive.getCurrentPosition());
        telemetry.addData("lineColorSensor_blue", lineColorSensor.blue());
        telemetry.addData("lineColorSensor_red", lineColorSensor.red());
        telemetry.addData("lineColorSensor_green", lineColorSensor.green());
        telemetry.addData("jewelArm_position", jewelArm.getPosition());
        telemetry.addData("grabberL", grabberL.getPosition());
        telemetry.addData("grabberR", grabberR.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

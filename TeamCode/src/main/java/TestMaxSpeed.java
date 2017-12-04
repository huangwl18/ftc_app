
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="TestMaxSpeed", group = "9367")
public class TestMaxSpeed extends LinearOpMode {

    private DcMotor LFDrive, RFDrive, LRDrive, RRDrive;
    private Servo jewelArm, grabberL, grabberR;



    @Override
    public void runOpMode() throws InterruptedException {

        LFDrive  = hardwareMap.get(DcMotor.class, "LFDrive");
        RFDrive = hardwareMap.get(DcMotor.class, "RFDrive");
        LRDrive  = hardwareMap.get(DcMotor.class, "LRDrive");
        RRDrive = hardwareMap.get(DcMotor.class, "RRDrive");

        RFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RRDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        jewelArm = hardwareMap.get(Servo.class, "jewelArm");
        grabberL = hardwareMap.get(Servo.class, "grabberL");
        grabberR = hardwareMap.get(Servo.class, "grabberR");

        jewelArm.setPosition(0);
        grabberL.setPosition(0.0594);
        grabberR.setPosition(0.98);

        waitForStart();


        LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long init = System.currentTimeMillis();

        while(System.currentTimeMillis() - init < 3000){
            LFDrive.setPower(1);
            RFDrive.setPower(1);
            LRDrive.setPower(1);
            RRDrive.setPower(1);
        }

        LFDrive.setPower(0);
        RFDrive.setPower(0);
        LRDrive.setPower(0);
        RRDrive.setPower(0);

        while(opModeIsActive()){
            telemetry.addData("LF max speed", LFDrive.getCurrentPosition()/3 );
            telemetry.addData("RF max speed", RFDrive.getCurrentPosition()/3 );
            telemetry.addData("LR max speed", LRDrive.getCurrentPosition()/3 );
            telemetry.addData("RR max speed", RRDrive.getCurrentPosition()/3 );
            telemetry.update();
        }




    }


}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


@Autonomous(name="Blue Right", group="Auto")
@Disabled
public class OpenCVTest extends LinearOpMode {
    
    OpenCvWebcam webcam;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightRear = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;

    //Other
    private DcMotor motorArm = null;
    private DcMotor motorWheel = null;
    private Servo armServo = null;

    private double PowerA = 0.80;
    int z = 0;

    private int target = 0;
    private double speed = 0.01;
    private int speed2 = 30;
    private int current;

    private boolean bLeft  = false;
    private boolean bMid  = false;
    private boolean bRight = false;

    public void resetEncoders(){
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void forward(int target) {
        // double startTime = runtime.milliseconds();
        //while(runtime.milliseconds() < startTime + waitTime && opModeIsActive()){
        resetEncoders();
        rightRear.setTargetPosition(target);
        leftRear.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        leftFront.setTargetPosition(target);

        rightRear.setPower(PowerA);
        leftRear.setPower(PowerA);
        rightFront.setPower(PowerA);
        leftFront.setPower(PowerA);
        while(opModeIsActive() && leftRear.isBusy()){
            //correction = checkDirection();
            //leftTankSteering(power + correction);
            //rightTankSteering(power - correction);
            telemetry.addData("encoder-rightRear", rightRear.getCurrentPosition() + "  busy=" + rightRear.isBusy());
            telemetry.addData("encoder-leftRear", leftRear.getCurrentPosition() + "  busy=" + leftRear.isBusy());
            telemetry.addData("encoder-rightRear", rightFront.getCurrentPosition() + "  busy=" + rightFront.isBusy());
            telemetry.addData("encoder-leftRear", leftFront.getCurrentPosition() + "  busy=" + leftFront.isBusy());
            telemetry.update();
        }
    }


    public void turn(int target) {
        resetEncoders();

        rightRear.setTargetPosition(target);
        leftRear.setTargetPosition(-target);
        rightFront.setTargetPosition(target);
        leftFront.setTargetPosition(-target);

        rightRear.setPower(PowerA);
        leftRear.setPower(PowerA);
        rightFront.setPower(PowerA);
        leftFront.setPower(PowerA);

        while(opModeIsActive() && leftRear.isBusy()){
            telemetry.addData("encoder-rightRear", rightRear.getCurrentPosition() + "  busy=" + rightRear.isBusy());
            telemetry.addData("encoder-leftRear", leftRear.getCurrentPosition() + "  busy=" + leftRear.isBusy());
            telemetry.addData("encoder-rightRear", rightFront.getCurrentPosition() + "  busy=" + rightFront.isBusy());
            telemetry.addData("encoder-leftRear", leftFront.getCurrentPosition() + "  busy=" + leftFront.isBusy());
            telemetry.update();
        }
    }

    public void leftTankSteering(double powerL){
        rightRear.setPower(powerL);
        rightFront.setPower(powerL);
    }

    public void rightTankSteering(double powerL){
        leftRear.setPower(powerL);
        leftFront.setPower(powerL);
    }


    public void forwardNC(int time){
        rightRear.setPower(PowerA);
        leftRear.setPower(PowerA);
        rightFront.setPower(PowerA);
        leftFront.setPower(PowerA);
        sleep(time);
    }

    public void left(int target) {
        resetEncoders();
        rightRear.setTargetPosition(-target);
        leftRear.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        leftFront.setTargetPosition(-target);

        rightRear.setPower(PowerA);
        leftRear.setPower(PowerA);
        rightFront.setPower(PowerA);
        leftFront.setPower(PowerA);
        while(opModeIsActive() && rightRear.isBusy()){
            telemetry.addData("encoder-rightRear", rightRear.getCurrentPosition() + "  busy=" + rightRear.isBusy());
            telemetry.addData("encoder-leftRear", leftRear.getCurrentPosition() + "  busy=" + leftRear.isBusy());
            telemetry.addData("encoder-rightRear", rightFront.getCurrentPosition() + "  busy=" + rightFront.isBusy());
            telemetry.addData("encoder-leftRear", leftFront.getCurrentPosition() + "  busy=" + leftFront.isBusy());
            telemetry.update();
        }
    }

    public void right(int target) {
        resetEncoders();
        rightRear.setTargetPosition(target);
        leftRear.setTargetPosition(-target);
        rightFront.setTargetPosition(-target);
        leftFront.setTargetPosition(target);

        rightRear.setPower(PowerA);
        leftRear.setPower(PowerA);
        rightFront.setPower(PowerA);
        leftFront.setPower(PowerA);
        while(opModeIsActive() && rightRear.isBusy()){
            telemetry.addData("encoder-rightRear", rightRear.getCurrentPosition() + "  busy=" + rightRear.isBusy());
            telemetry.addData("encoder-leftRear", leftRear.getCurrentPosition() + "  busy=" + leftRear.isBusy());
            telemetry.addData("encoder-rightRear", rightFront.getCurrentPosition() + "  busy=" + rightFront.isBusy());
            telemetry.addData("encoder-leftRear", leftFront.getCurrentPosition() + "  busy=" + leftFront.isBusy());
            telemetry.update();
        }
    }

    public void backward(int target) {
        resetEncoders();
        rightRear.setTargetPosition(-target);
        leftRear.setTargetPosition(-target);
        rightFront.setTargetPosition(-target);
        leftFront.setTargetPosition(-target);

        rightRear.setPower(0.30);
        leftRear.setPower(0.30);
        rightFront.setPower(0.30);
        leftFront.setPower(0.30);
        while(opModeIsActive() && rightRear.isBusy()){
            telemetry.addData("encoder-rightRear", rightRear.getCurrentPosition() + "  busy=" + rightRear.isBusy());
            telemetry.addData("encoder-leftRear", leftRear.getCurrentPosition() + "  busy=" + leftRear.isBusy());
            telemetry.addData("encoder-rightRear", rightFront.getCurrentPosition() + "  busy=" + rightFront.isBusy());
            telemetry.addData("encoder-leftRear", leftFront.getCurrentPosition() + "  busy=" + leftFront.isBusy());
            telemetry.update();
        }
    }
    public void turnOffMotors(){
        rightRear.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
    }
    
    @Override
    public void runOpMode() {
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorWheel = hardwareMap.get(DcMotor.class, "motorWheel");
        armServo = hardwareMap.get(Servo.class, "armServo");

        //  Motor Direction
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.REVERSE);
        motorWheel.setDirection(DcMotor.Direction.FORWARD);

        // Encoders
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorArm.setTargetPosition(0);
        rightRear.setTargetPosition(0);
        leftRear.setTargetPosition(0);
        rightFront.setTargetPosition(0);
        leftFront.setTargetPosition(0);

        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
        SkystoneDetector detector = new SkystoneDetector(telemetry);
        webcam.setPipeline(detector);
        //webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
         {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Error", "ERROR");
            }
        });

        
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        webcam.stopStreaming();

        //Move arm up
        motorArm.setTargetPosition(300);
        motorArm.setPower(.5);
        armServo.setPosition(0.0);
        while(opModeIsActive() && motorArm.isBusy()){}
        motorArm.setPower(0);
        sleep(100);

        //Move to carousel
        forward(400);
        turnOffMotors();
        sleep(500);
        turn(1250);
        backward(1600);
        turnOffMotors();
        sleep(500);
        motorWheel.setPower(.3);
        sleep(4000);
        motorWheel.setPower(0);

        //Move to drop box
        right(2300);
        turnOffMotors();
        sleep(250);
        //turn(-1200);
        turnOffMotors();
        sleep(500);
        // forward(1600);
        // turnOffMotors();
        // sleep(500);
        //turn(1200);
        turnOffMotors();
        sleep(500);
        switch (detector.getLocation()) {
            case LEFT:
                motorArm.setTargetPosition(2900);
                motorArm.setPower(.5);
                while (opModeIsActive() && motorArm.isBusy()) {
                }
                motorArm.setPower(0);
                PowerA = 0.5;
                forward(1850);

                break;
            case RIGHT:
                motorArm.setTargetPosition(580);
                motorArm.setPower(.5);
                while (opModeIsActive() && motorArm.isBusy()) {
                }
                motorArm.setPower(0);
                forward(1800);

                break;
            case MIDDLE:
                motorArm.setTargetPosition(1700);
                motorArm.setPower(.5);
                while (opModeIsActive() && motorArm.isBusy()) {
                }
                motorArm.setPower(0);
                forward(1800);
                turnOffMotors();
                sleep(1000);
                PowerA = 0.8;
                armServo.setPosition(.24);
                sleep(500);
        }

        //Park in storage
        backward(1900);
        turnOffMotors();
        sleep(500);
        left(800);
        turnOffMotors();
        sleep(500);
        backward(150);
        turnOffMotors();
        sleep(500);
        armServo.setPosition(0.05);
        motorArm.setTargetPosition(-20);
        motorArm.setPower(.5);
        while (opModeIsActive() && motorArm.isBusy()) {}
        motorArm.setPower(0);

        sleep(1000);
        stop();
    }
}
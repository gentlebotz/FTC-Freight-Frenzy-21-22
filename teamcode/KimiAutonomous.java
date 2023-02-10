package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.tree.JCTree;

import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Vector;

@Autonomous (name = "KimiRoadRunner", group = "AutoRR")
public class KimiAutonomous extends LinearOpMode {

    /*
    Robot Vars
     */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightRear = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;

    //Other
    private DcMotor motorArm = null;
    private DcMotor motorWheel = null;
    private Servo armServo = null;
    private Servo linearServo = null;
    private double linearPosition;

    private int targetHigh = 2700;
    private int targetMid = 1600;
    private int targetLow = 625;


    public static double mapRange(double a1, double a2, double b1, double b2, double s){
        return b1 + ((s - a1)*(b2 - b1))/(a2 - a1);
    }

    /*
    OpenCV Vars
    */
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    double CrLowerUpdate = 150;
    double CbLowerUpdate = 120;
    double CrUpperUpdate = 255;
    double CbUpperUpdate = 255;

    double lowerruntime = 0;
    double upperruntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0, 123, 124);
    public static Scalar scalarUpperYCrCb = new Scalar(70, 135, 135);


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorWheel = hardwareMap.get(DcMotor.class, "motorWheel");
        armServo = hardwareMap.get(Servo.class, "armServo");
        linearServo = hardwareMap.get(Servo.class, "linearServo");

        //  Motor Direction
        motorArm.setDirection(DcMotor.Direction.REVERSE);
        motorWheel.setDirection(DcMotor.Direction.FORWARD);

        // Encoders
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setTargetPosition(0);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /*
        Initialize EasyOpenCV
         */

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        PipeLine myPipeline;
        webcam.setPipeline(myPipeline = new PipeLine());
        // Configuration of Pipeline
        myPipeline.ConfigurePipeline(30, 30,30,30,  CAMERA_WIDTH, CAMERA_HEIGHT);
        myPipeline.ConfigureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.ConfigureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        /*
        Initialize RoadRunner
         */

        Pose2d startPose = new Pose2d(-34, -60, Math.toRadians(90));

        TrajectorySequence trajToCarrousel = drive.trajectorySequenceBuilder(startPose)
                .forward(1)
                .build();

        drive.setPoseEstimate(startPose);

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        while (opModeIsActive()) {
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

//            if (myPipeline.getRectArea() > 1000) {
            if (myPipeline.getRectMidpointX() > 400) {
                //AUTONOMOUS_C();
                webcam.stopStreaming();
                telemetry.addData("Marker location: ", "Right");
                telemetry.update();
                if (!isStopRequested()) {
                    drive.followTrajectorySequence();
                }
            } else if (myPipeline.getRectMidpointX() > 200) {
                //AUTONOMOUS_B();
                webcam.stopStreaming();
                telemetry.addData("Marker location: ", "Mid");
                telemetry.update();
                if (!isStopRequested()) {
                    drive.followTrajectorySequence();
                }
            } else {
                //AUTONOMOUS_A();
                webcam.stopStreaming();
                telemetry.addData("Marker location: ", "Left");
                telemetry.update();
                if (!isStopRequested()) {
                    drive.followTrajectorySequence();
                }
            }
            motorArm.setTargetPosition(0);
            motorArm.setPower(.8);
            linearServo.setPosition(mapRange(3300, -100, .1, .50, 0));
            armServo.setPosition(0.05);
            while (opModeIsActive() && motorArm.isBusy() && linearServo.getPosition() > .12) {
            }
            sleep(100);
            motorArm.setPower(0);
            break;
        }
    }
}

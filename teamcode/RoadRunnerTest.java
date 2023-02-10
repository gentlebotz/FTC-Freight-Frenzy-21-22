package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(group = "drive")
public class RoadRunnerTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Other
    private DcMotor motorArm = null;
    private DcMotor motorWheel = null;
    private Servo armServo = null;
    private Servo linearServo = null;

    public static double mapRange(double a1, double a2, double b1, double b2, double s) {
        return b1 + ((s - a1) * (b2 - b1)) / (a2 - a1);
    }

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

        Pose2d startPose = new Pose2d(-34, 60, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    motorArm.setTargetPosition(350);
                    motorArm.setPower(.5);
                    armServo.setPosition(0.0);})
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {motorArm.setPower(0);})

                //Go to carousel
                .lineToLinearHeading(new Pose2d(-57,55, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () ->
                        motorWheel.setPower(.25))
                .waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () ->
                        motorWheel.setPower(0))

                //Move to wobble tower
                .strafeTo(new Vector2d(-60, 35))
                .splineToConstantHeading(new Vector2d(-29, 24), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    motorArm.setTargetPosition(650);
                    motorArm.setPower(.8);
                })

                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    armServo.setPosition(.24);
                    motorArm.setPower(0);
                })
                .waitSeconds(2)

                //Drive to warehouse
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(-50, 25))
                .splineToConstantHeading(new Vector2d(-58, 30), Math.toRadians(90))
                //.resetVelConstraint()
                .splineToConstantHeading(new Vector2d(5, 67), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45, 69), Math.toRadians(0))

                .waitSeconds(3)
                .lineTo(new Vector2d(0, 55))
                .lineToLinearHeading(new Pose2d(-34, 60, Math.toRadians(-90)))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}

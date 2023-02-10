/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.robotcontroller.external.samples;

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

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name="RedWarehouse", group="Linear Opmode")
@Disabled
public class RedWarehouse extends LinearOpMode {
   private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };


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

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
     double leftPower, rightPower;
    double offset;
    
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
    
    public void dropDuck(){
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
        left(1600);
        turnOffMotors();
        sleep(500);
        
        //Move to drop box
        turnOffMotors();
        sleep(500);
        if(bLeft) {
            motorArm.setTargetPosition(400);
            motorArm.setPower(.5);
            while(opModeIsActive() && motorArm.isBusy()){}
            motorArm.setPower(0);
            forward(700);
        }
        else if(bMid) {
            motorArm.setTargetPosition(1600);
            motorArm.setPower(.5);
            while(opModeIsActive() && motorArm.isBusy()){}
            motorArm.setPower(0);
            forward(700);
        }
        else if(bRight) {
            motorArm.setTargetPosition(2600);
            motorArm.setPower(.5);
            while(opModeIsActive() && motorArm.isBusy()){}
            motorArm.setPower(0);
            PowerA = 0.5;
            forward(1100);
        }
        turnOffMotors();
        sleep(1000);
        PowerA = 0.8;
        armServo.setPosition(.24);
        sleep(500);
        
        //Park in storage
        backward(200);
        turnOffMotors();
        sleep(500);
        turn(-1250);
        turnOffMotors();
        sleep(500);
        PowerA = 1;
        forward(4000);
        turnOffMotors();
        sleep(500);
        PowerA = 0.8;
        right(300);
        sleep(500);
        armServo.setPosition(0.05);
        motorArm.setTargetPosition(-30);
        motorArm.setPower(.5);
        while(opModeIsActive() && motorArm.isBusy()){}
        motorArm.setPower(0);
        
        sleep(1000);
        tfod.shutdown();
        stop();
    }
        
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "VUFORIA_KEY";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        
        // BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // parameters.mode                = BNO055IMU.SensorMode.IMU;
        // parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        // parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // parameters.loggingEnabled      = true;
        
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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
        
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0/9.0);
        }

        // imu = hardwareMap.get(BNO055IMU.class, "imu");

        // imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        //make sure the imu gyro is calibrated before continuing.
        // while (!isStopRequested() && !imu.isGyroCalibrated())
        // {
        //     sleep(50);
        //     idle();
        // }

        // offset = getAngle();
        
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null && z <= 0) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null && z <= 0) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());

                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      boolean isDuckDetected = false;     //  ** ADDED **
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                          recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;

                        // check label to see if the camera now sees a Duck         ** ADDED **
                        if (recognition.getLabel().equals("Duck") && z <= 0) {            //  ** ADDED **
                             telemetry.addData("Object Detected", "Duck");
                                                                               //  ** ADDED **
                             if (recognition.getLeft() >= 0 && recognition.getLeft() <= 300 && z <= 0){
                                bLeft = true; 
                                telemetry.addData("Barcode", "Left");
                                z=z+1;
                                    
                                dropDuck();
                                
                             }
                             
                             if (recognition.getLeft() >= 300 && recognition.getLeft() <= 700 && z <= 0){
                                bMid = true;
                                telemetry.addData("Barcode", "Mid");
                                z=z+1;
                                
                                dropDuck();
                             }
                             
                             if (recognition.getLeft() >= 700 && recognition.getLeft() <= 2000 && z <= 0){
                                bRight = true;
                                telemetry.addData("Barcode", "Right");
                                z=z+1;
                                
                                dropDuck();
                             }
                         } else { 
                             if(z <= 0) {
                                isDuckDetected = false;                            //  ** ADDED **
                                dropDuck();
                             }
                         }                                          
                      }
                      telemetry.update();
                    }
                    z=z+1;
                }
            }
        }
        
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    } 

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.52f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .015;

        angle = getAngle() - offset;

        if (angle == -55 )
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(double degrees, double power)
    {
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double  leftPower, rightPower;
        degrees  = degrees - 25;
        
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        rightTankSteering(leftPower);
        leftTankSteering(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        rightRear.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }   
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.currentThreadTimeMillis;
import static android.os.SystemClock.setCurrentTimeMillis;
import static android.os.SystemClock.sleep;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "DrivingFF", group = "Iterative Opmode")
//@Disabled
public class DrivingFreightFrenzy extends OpMode {
    // Declare OpMode members.

    //Wheels
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

    //Variables
    private double power = 1;
    private double power2 = .4;
    private boolean turboStop = true;
    private boolean turbo = false;
    private boolean handOpen = false;
    private boolean handButtonPressed = true;
    private boolean manualServo = true;
    private boolean manualServoPressed = true;
    private boolean freeMode = false;
    private boolean freeModePressed = true;

    private boolean topHoogte = false;
    private boolean midHoogte = false;
    private boolean lowHoogte = false;
    private boolean grndHoogte = false;

    private int target = 0;
    private int speed2 = 120;
    private int current;
    private double linearPosition;

    public static double mapRange(double a1, double a2, double b1, double b2, double s){
        return b1 + ((s - a1)*(b2 - b1))/(a2 - a1);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override

    public void init() {
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
        linearServo = hardwareMap.get(Servo.class, "linearServo");

        //  Motor Direction
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.REVERSE);
        motorWheel.setDirection(DcMotor.Direction.FORWARD);

        // Encoders
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double G1leftStickY = -gamepad1.left_stick_y;
        double G1leftStickX = -gamepad1.left_stick_x;
        double G1rightStickX = gamepad1.right_stick_x;
        double G2rightStickY = -gamepad2.right_stick_y;
        double G2rightStickX = -gamepad2.right_stick_x;
        double G2leftStickY = -gamepad2.left_stick_y;

        rightRear.setPower(power2 * (G1leftStickY + -G1leftStickX + 1.2 * -G1rightStickX));
        leftRear.setPower(power2 * (G1leftStickY + G1leftStickX + 1.2 * G1rightStickX));
        rightFront.setPower(power2 * (G1leftStickY + G1leftStickX + 1.2 * -G1rightStickX));
        leftFront.setPower(power2 * (G1leftStickY + -G1leftStickX + 1.2 * G1rightStickX));

        // Open & Close arm hand
        if(gamepad2.a && !handButtonPressed){
            handButtonPressed = true;
            handOpen = !handOpen;
            armServo.setPosition(handOpen ? 0.24 : 0.05);
        }

        else if (!gamepad2.a) {
            handButtonPressed = false;
        }

        // Ducky wheel
        if (!gamepad2.x || !gamepad2.b){
            motorWheel.setPower(0);
        }

        if(gamepad2.x) {
            motorWheel.setPower(.42);
        }

        if(gamepad2.b) {
            motorWheel.setPower(-.42);
        }

        //Manual linear servo toggle
        if(gamepad2.right_bumper && !manualServoPressed){
            manualServoPressed = true;
            manualServo = !manualServo;
        }

        else if (!gamepad2.right_bumper) {
            manualServoPressed = false;
        }

        if(gamepad2.left_bumper && !freeModePressed){
            freeModePressed = true;
            freeMode = !freeMode;
        }

        else if (!gamepad2.left_bumper) {
            freeModePressed = false;
        }

        // Arm with encoders
        current =  motorArm.getCurrentPosition();

        int current = motorArm.getCurrentPosition();
        int incr = (int)(-gamepad2.left_stick_y * speed2);

        target += incr;

        if(target > 3800)
        {
            target = 3800;
        }

        if(target < -100  && !freeMode)
        {
            target = -100;
        }

        if(Math.abs(target-current) > speed2)
        {
            target = current + (int)Math.signum(target-current)*speed2;
        }

        linearPosition = mapRange(3300, -100, .1, .50, target);
        if(!manualServo){
            linearServo.setPosition(linearPosition);
        }
        if(manualServo){
            if(gamepad2.right_stick_y < -.1 && linearServo.getPosition() > 0.1) {
                linearServo.setPosition(linearServo.getPosition() + 0.001);
            }
            if(gamepad2.right_stick_y > .1) {
                linearServo.setPosition(linearServo.getPosition() - 0.001);
            }
        }

        //if(topHoogte && midHoogte && lowHoogte && grndHoogte){
            motorArm.setTargetPosition(target);
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(motorArm.isBusy()){
                motorArm.setPower(0.4);
            }
        //}

         if(!motorArm.isBusy()){
             motorArm.setPower(0);
         }

//        if(gamepad2.dpad_up && topHoogte){
//            topHoogte = false;
//            motorArm.setTargetPosition(3000);
//            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorArm.setPower(0.5);
//        }
//
//        if(!gamepad2.dpad_up && !motorArm.isBusy()){
//            topHoogte = true;
//        }
//
//        if(gamepad2.dpad_left && midHoogte){
//            midHoogte = false;
//            motorArm.setTargetPosition(1700);
//            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorArm.setPower(0.5);
//        }
//
//        if(!gamepad2.dpad_left && !motorArm.isBusy()){
//            midHoogte = true;
//        }
//
//        if(gamepad2.dpad_right && lowHoogte){
//            lowHoogte = false;
//            motorArm.setTargetPosition(700);
//            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorArm.setPower(0.5);
//        }
//
//        if(!gamepad2.dpad_right && !motorArm.isBusy()){
//            lowHoogte = true;
//        }
//
//        if(gamepad2.dpad_down && grndHoogte){
//            grndHoogte = false;
//            motorArm.setTargetPosition(0);
//            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorArm.setPower(0.5);
//        }
//
//        if(!gamepad2.dpad_down && !motorArm.isBusy()){
//            grndHoogte = true;
//        }

        // Turbo mode
        if (gamepad1.left_bumper && turboStop) {
            turboStop = false;
            turbo = !turbo;
            //power2 = turbo ? 1 : 0.5;
        }

        else if (!gamepad1.left_bumper) {
            turboStop = true;
        }

        // Telemetry
        telemetry.addData("Power mode: ", turbo ? "Turbo" : "No turbo");
        telemetry.addData("Servo mode: ", manualServo ? "Manual" : "Automatic");
        telemetry.addData("Arm limits: ", freeMode ? "Off" : "On");
        telemetry.addData("runtime", runtime);

        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        rightRear.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
    }

}
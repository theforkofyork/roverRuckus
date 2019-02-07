package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Objects;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Motor channel:  Manipulator drive motor:  "arm motor"
 * Servo channel:  Servo to open left claw:  "left claw"
 * Servo channel:  Servo to open right claw: "right claw"
 */
public class LBHW {
    /* Public OpMode members. */
    BNO055IMU imu;
    DcMotor in;
    DcMotor extend;
    //  DcMotor lift;
    Servo dump;
    Servo g;
    Servo tilt;
    Servo wheel;

    public DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;
    public DcMotor hang;
    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public LBHW() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {


        // Save reference to Hardware map
        hwMap = ahwMap;
        leftFrontWheel = hwMap.dcMotor.get("LF");
        leftBackWheel = hwMap.dcMotor.get("LB");
        rightFrontWheel = hwMap.dcMotor.get("RF");
        rightBackWheel = hwMap.dcMotor.get("RB");
        hang = hwMap.dcMotor.get("hang");
        in = hwMap.dcMotor.get("in");
        //  imu = new MasqAdafruitIMU("IMU", hwMap);
        // Define and Initialize Motors
        rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt = hwMap.servo.get("tilt");
        g = hwMap.servo.get("g");
        dump = hwMap.servo.get("dump");
        wheel = hwMap.servo.get("wheel");
        extend = hwMap.dcMotor.get("extend");

        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        g.setPosition(.1);
    //    dump.setPosition(.137);
        encoders();
        wheel.setPosition(.9);

    }




    public void setStrafePower(String Direction, double Speed) {
        if(Objects.equals(Direction, "Left"))
        {
            rightFrontWheel.setPower(Speed);
            rightBackWheel.setPower(-Speed);
            leftFrontWheel.setPower(-Speed);
            leftBackWheel.setPower(Speed);
        }
        if(Objects.equals(Direction, "Right"))
        {
            rightFrontWheel.setPower(-Speed);
            rightBackWheel.setPower(Speed);
            leftFrontWheel.setPower(Speed);
            leftBackWheel.setPower(-Speed);
        }
    }
    public void encoders() {
        leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoder() {
        leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void right(double power) {
        rightBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
    }
    public void left(double power) {
        leftFrontWheel.setPower(power);
        leftBackWheel.setPower(power);
    }
}



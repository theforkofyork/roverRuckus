package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
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
    public static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    public DcMotor lift;
    public DcMotor in;
    public DcMotor extend;
    //  DcMotor lift;
    public Servo dump;
    public Servo g;
    public Servo marker;
    Servo block;
    public DigitalChannel touch;
    //public Servo tilt;
    public Servo wheel;
    DcMotor hang;
    boolean slow = false;
    boolean dumped = false;
    boolean extending = false;
    boolean dumping;
    double gUpKp = 0.005;
    int gUpTolerance = 8;
    double gDownKp = 0.0032;
    int gDownTolerance = 4;
    double eUpKp = 0.004;
    int eUpTolerance = 8;
    double eDownKp = 0.0018;
    int eDownTolerance = 5;
    public double dumpIdle = .18;
    public double dumpPos = .82;
    boolean hanger = false;
    boolean retract = false;
    boolean retracting = false;
    boolean lifting = false;
    boolean retractThread = false;
    int i = 0;
    int i2 = 0;





   public double tiltUp = .9;
   public double tiltDown = .48;

  public  double gClosed = .9;
    public double gOpen = .28;

    boolean isWaiting = false;
    long waitTime = 0;


    public ServoImplEx tilt;
    HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public LBHW() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Save reference to Hardware map
        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        leftFrontWheel = hwMap.dcMotor.get("LF");
        leftBackWheel = hwMap.dcMotor.get("LB");
        rightFrontWheel = hwMap.dcMotor.get("RF");
        rightBackWheel = hwMap.dcMotor.get("RB");
        lift = hwMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        in = hwMap.dcMotor.get("in");
        in.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend = hwMap.dcMotor.get("extend");
        // tilt = hardwareMap.servo.get("tilt");
        g = hwMap.servo.get("g");
        dump = hwMap.servo.get("dump");
        hang = hwMap.dcMotor.get("hang");
        wheel = hwMap.servo.get("wheel");
        tilt = hwMap.get(ServoImplEx.class, "tilt");
        touch = hwMap.get(DigitalChannel.class, "touch");
        marker = hwMap.servo.get("marker");
        block = hwMap.servo.get("block");


        touch.setMode(DigitalChannel.Mode.INPUT);

        in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // in.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blinkinLedDriver.setPattern(pattern);
        g.setPosition(gClosed);
        // dump.setPosition(dumpIdle);
        wheel.setPosition(.9);
        marker.setPosition(1);
        runtime.reset();
        block.setPosition(0);

        encoders();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
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
        in.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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



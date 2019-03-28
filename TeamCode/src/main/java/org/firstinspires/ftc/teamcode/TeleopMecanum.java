package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Tele", group="LANbros")
public class TeleopMecanum extends OpMode {

    private ScoringLift lifter;

    public TeleopMecanum() {

    }


    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    DcMotor lift;
    DcMotor in;
    DcMotor extend;
    Servo marker;
    //  DcMotor lift;
    Servo dump;
    Servo g;
    Servo block;
    DigitalChannel touch;
    //Servo tilt;
    Servo wheel;
    DcMotor hang;
    boolean slow = false;
    boolean dumped = false;
    boolean extending = false;
    boolean dumping;
    double gUpKp = 0.0028;
    int gUpTolerance = 8;
    double gDownKp = 0.0025;
    int gDownTolerance = 4;
    double eUpKp = 0.004;
    int eUpTolerance = 8;
    double eDownKp = 0.0018;
    int eDownTolerance = 5;
    double dumpIdle = .18;
    double dumpPos = .82;
    boolean hanger = false;
    boolean retract = false;
    boolean retracting = false;
    boolean lifting = false;
    boolean retractThread = false;
    int i = 0;
    int i2 = 0;

    private ElapsedTime     runtime = new ElapsedTime();



    double tiltUp = .44;
    double tiltDown = 0;

    double gClosed = .74;
    double gOpen = .11;

    boolean isWaiting = false;
    long waitTime = 0;

    boolean isWaiting2  = false;
    long waitTime2 = 0;

    boolean lowering = false;
    boolean liftable = false;


    ServoImplEx tilt;

    @Override
    public void init() {

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        leftFrontWheel = hardwareMap.dcMotor.get("LF");
        leftBackWheel = hardwareMap.dcMotor.get("LB");
        rightFrontWheel = hardwareMap.dcMotor.get("RF");
        rightBackWheel = hardwareMap.dcMotor.get("RB");
        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        in = hardwareMap.dcMotor.get("in");
       // in.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend = hardwareMap.dcMotor.get("extend");
        // tilt = hardwareMap.servo.get("tilt");
        g = hardwareMap.servo.get("g");
        dump = hardwareMap.servo.get("dump");
        hang = hardwareMap.dcMotor.get("hang");
        wheel = hardwareMap.servo.get("wheel");
        tilt = hardwareMap.get(ServoImplEx.class, "tilt");
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        marker = hardwareMap.servo.get("marker");
        block = hardwareMap.servo.get("block");


        touch.setMode(DigitalChannel.Mode.INPUT);

        in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // in.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter = new ScoringLift(lift, gUpKp, gUpTolerance, gDownKp, gDownTolerance);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("status", "loop test... waiting for start");
        pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
        blinkinLedDriver.setPattern(pattern);
        telemetry.update();


    }


    public void start() {
        tilt.setPosition(tiltUp);
        g.setPosition(gClosed);
        // tilt.setPosition(tiltDown);
        dump.setPosition(dumpIdle);
        wheel.setPosition(.9);
        //extender.setLiftSetpoint(80);
        runtime.reset();
        marker.setPosition(1);
        block.setPosition(1);
    }


    @Override
    public void loop() {

        if (runtime.seconds() < 90 && !retracting) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        } else if (runtime.seconds() > 90 && runtime.seconds() < 105 && !retracting) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        } else if (runtime.seconds() > 105 && runtime.seconds() < 120 && !retracting) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        } else if (runtime.seconds() > 120 && !retracting) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }


       // Thread.currentThread().setPriority(Thread.MAX_PRIORITY);


        float Ch1 = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
        float Ch3 = gamepad1.left_stick_y *  Math.abs(gamepad1.left_stick_y);
        float Ch4 = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);


        // note that if y equal -1 then joystick is pushed all of the way forward.
        double rightfront = Ch3 - Ch1 - Ch4;
        double rightback = Ch3 - Ch1 + Ch4;
        double leftfront = Ch3 + Ch1 + Ch4;
        double leftback = Ch3 + Ch1 - Ch4;


// Equation for Drive, Left Stick when pushed forward sends robot forward, Left Stick when sideways controls strafing, Right stick when pushed left controls turning

        // clip the right/left values so that the values never exceed +/- 1
        rightback = Range.clip(rightback, -1, 1);
        leftback = Range.clip(leftback, -1, 1);
        rightfront = Range.clip(rightfront, -1, 1);
        leftfront = Range.clip(leftfront, -1, 1);


        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        rightfront = (float) scaleInput(rightfront);
        leftfront = (float) scaleInput(leftfront);
        rightback = (float) scaleInput(rightback);
        leftback = (float) scaleInput(leftback);


        // write the values to the motors


        rightFrontWheel.setPower(rightfront);
        leftFrontWheel.setPower(leftfront);
        rightBackWheel.setPower(rightback);
        leftBackWheel.setPower(leftback);


        if (gamepad1.back) {
            hanger = true;
        } else if (gamepad1.start) {
            hanger = false;
        }

        if (gamepad1.left_bumper && hanger) {
            hang.setPower(1);
        } else if (gamepad1.left_trigger > .25 && hanger) {
            hang.setPower(-1);
        } else {
            hang.setPower(0);
        }

        if (hanger) {
        }


        if (gamepad1.right_bumper) {
            in.setPower(1);
        } else if (gamepad1.right_trigger > .25 && !retracting) {
            in.setPower(-1);
        } else if (gamepad1.right_trigger < .25 && !retracting) {
            in.setPower(0);
        }

        if (gamepad1.x) {
            dump.setPosition(dumpPos);
            dumped = true;
        } else if (gamepad1.b) {
            dump.setPosition(dumpIdle);
            dumped = false;
        }
        if (gamepad1.dpad_up) {
            extending = true;
            retracting = false;
            liftable = false;
            extend.setPower(1);
        } else if (gamepad1.dpad_down) {
            extending = false;
            // in.setPower(-1);
            if (!retract || !retracting) {
                retract = true;
                retracting = true;
                i = 0;

            }
        } else if (!retracting || !retract) {
            extend.setPower(0);
        }
        retract();


        if (gamepad1.a) {
            tilt.setPosition(tiltDown);
        }

        if (gamepad1.y) {
            {
                tilt.setPosition(tiltUp);
            }
        }
        if (gamepad1.dpad_left) {
            g.setPosition(gClosed);
        }
        if (gamepad1.dpad_right) {
            g.setPosition(gOpen);
        }
        if (gamepad1.right_stick_button) {
            extending = true;
            hanger = false;
            slow = true;
            tilt.setPosition(tiltUp);
            //raise();
            g.setPosition(gClosed);
        }
        if (gamepad1.left_stick_button) {
                hanger = true;
        }


        if (gamepad1.left_bumper && !hanger) {
            lowering = true;
            retracting = false;
            dump.setPosition(dumpIdle);

            if (lifting) {
                tilt.setPosition(tiltUp);
                g.setPosition(gClosed);
                extend.setPower(1);
                lifting = false;
            }
            if (lift.getCurrentPosition() <= 10) {
                lifter.relinquish();
                lowering = false;
            }
           if (!dumped) {
                lifter.setLiftSetpoint(0);
           }
        } else if (gamepad1.left_trigger > .25 && !hanger) {
            retracting = false;
            dumped = false;
            if (lifting && !dumped) {
                dump.setPosition(dumpIdle);
            }
            lifting = true;
            lowering = false;
            lift();
        }

        if (lowering && dumped) {
           lowerLift();
        }


        double extendBack = 950 ;
        double inStopPos = 530;
        if (retracting && extend.getCurrentPosition() <= extendBack) {
            tilt.setPosition(tiltUp);
        } else if (retracting && extend.getCurrentPosition() > extendBack) {
            tilt.setPosition(tiltDown);
        }
        if (retracting) {
            dump.setPosition(.125);

        }
        if (retracting && extend.getCurrentPosition() <= 870) {
            g.setPosition(gOpen);
        }

        lifter.update();

        if (retracting && !touch.getState()) {
            g.setPosition(gOpen);
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        }
        if (extending && !retracting){
            g.setPosition(gClosed);
        }
        if (extending && !retracting && extend.getCurrentPosition() > extendBack) {
            tilt.setPosition(tiltDown);
        }


        telemetry.addData("lift pos", lift.getCurrentPosition());
        telemetry.addData("hang pos", hang.getCurrentPosition());
        telemetry.addData("RB", rightBackWheel.getCurrentPosition());
        telemetry.addData("Horizontal Omni", rightFrontWheel.getCurrentPosition()/4);
        telemetry.addData("LB", leftBackWheel.getCurrentPosition());
        telemetry.addData("LF", leftFrontWheel.getCurrentPosition());
        telemetry.addData("extend", extend.getCurrentPosition());
        telemetry.addData("touched", touch.getState());

    }


    @Override
    public void stop() {
        lifter.close();
        tilt.setPwmDisable();
    }


    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, .2, .21, .22, .23, 0.24, .25, .26, .27, .28, .29,
                0.30, .32, 0.36, .40, 0.43, .47, 0.50, .53, .55, 0.60, .63, .65, .69, 0.72, .74, .77, .8, .83, 0.85, .87, .9, .93, .96, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 41.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 41) {
            index = 41;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.,
        return dScale;
    }

    public void lift() {
        lowering = false;
        lifting = true;
        in.setPower(0);
        tilt.setPosition(tiltDown);
        dump.setPosition(dumpIdle);
        lifter.setLiftSetpoint(670);
        dumped = false;

    }

    public void retract() {

        if (retracting) {
            in.setPower(-1);
            if (!touch.getState() && retracting) {
                liftable = true;
                extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                in.setPower(-1);
                g.setPosition(gOpen);
                if (!isWaiting)
                    waitTime = System.currentTimeMillis();
                isWaiting = true;
            }
            else if (i < 1) {
                extend.setPower(-1);

            }

        }
        if (System.currentTimeMillis() - waitTime > 300 && isWaiting && !lifting && retracting) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            in.setPower(0);
            if (liftable) {
                lift();
            }
            retracting = false;
            isWaiting = false;
            liftable = false;
            i++;
        }

    }




    public void sleep(double time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void lowerLift() {
        if (dumped) {
            dump.setPosition(.18);

            if (!isWaiting2) {
                waitTime2 = System.currentTimeMillis();
                isWaiting2 = true;
            }
            if (System.currentTimeMillis() - waitTime2 > 200 && isWaiting2) {
                lifter.setLiftSetpoint(0);
                isWaiting2 = false;
                dumped = false;
                lifting = false;
                lowering = false;
            }

        }
    }

}
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

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevExtensions2;


@TeleOp(name="TeleCrater", group="LANbros")
public class TeleCrater extends OpMode {

    private ScoringLift lifter;
    private Hang hange;

    public TeleCrater() {

    }


    private static ExpansionHubMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    ExpansionHubMotor lift;
    ExpansionHubMotor in;
    ExpansionHubMotor extend;
    Servo marker;
    //  DcMotor lift;
    Servo dump;
    Servo g;
    Servo block;
    DigitalChannel touch;
    //Servo tilt;
    Servo wheel;
    ExpansionHubMotor hang;
    boolean slow = false;
    boolean dumped = false;
    boolean extending = false;
    boolean dumping;
    double gUpKp = 0.0032;
    int gUpTolerance = 4;
    double gDownKp = 0.0025;
    int gDownTolerance = 4;
    double eUpKp = 0.004;
    int eUpTolerance = 8;
    double eDownKp = 0.0018;
    int eDownTolerance = 5;
    double dumpIdle = .17;
    double dumpPos = .82;
    boolean hanger = false;
    boolean retract = false;
    boolean retracting = false;
    boolean lifting = false;
    boolean retractThread = false;
    int i = 0;
    int i2 = 0;

    boolean intaking = false;

    private ElapsedTime     runtime = new ElapsedTime();



    double tiltUp = .9;
    double tiltDown = .48;

    double gClosed = .92;
    double gOpen = .28;

    boolean isWaiting = false;
    long waitTime = 0;

    boolean isWaiting3 = false;
    long waitTime3 = 0;

    boolean isWaiting2  = false;
    long waitTime2 = 0;

    boolean lowering = false;
    boolean liftable = false;
    boolean button = false;
    boolean touched = false;

    double old = 0;
    double ratio = .4;

    ExpansionHubEx hub3, hub10;


    ServoImplEx tilt;

    @Override
    public void init() {
        RevExtensions2.init();
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        leftFrontWheel = (ExpansionHubMotor) hardwareMap.dcMotor.get("LF");
        leftBackWheel = (ExpansionHubMotor) hardwareMap.dcMotor.get("LB");
        rightFrontWheel = (ExpansionHubMotor) hardwareMap.dcMotor.get("RF");
        rightBackWheel = (ExpansionHubMotor) hardwareMap.dcMotor.get("RB");
        lift = (ExpansionHubMotor) hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        in = (ExpansionHubMotor) hardwareMap.dcMotor.get("in");
        // in.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend = (ExpansionHubMotor) hardwareMap.dcMotor.get("extend");
        // tilt = hardwareMap.servo.get("tilt");
        g = hardwareMap.servo.get("g");
        dump = hardwareMap.servo.get("dump");
        hang = (ExpansionHubMotor) hardwareMap.dcMotor.get("hang");
        wheel = hardwareMap.servo.get("wheel");
        tilt = hardwareMap.get(ServoImplEx.class, "tilt");
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        marker = hardwareMap.servo.get("marker");
        block = hardwareMap.servo.get("block");

        hub3 =hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 3");
        hub10 = hardwareMap.get(ExpansionHubEx.class, "hub10");

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
        hange = new Hang(hang, gUpKp, gUpTolerance, gDownKp, gDownTolerance);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("status", "loop test... waiting for start");
        // pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
        //blinkinLedDriver.setPattern(pattern);
        telemetry.update();


    }


    public void start() {
        tilt.setPosition(tiltDown);
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

      /*  if (runtime.seconds() < 90 && !retracting) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        } else if (runtime.seconds() > 90 && runtime.seconds() < 105 && !retracting) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        } else if (runtime.seconds() > 105 && runtime.seconds() < 120 && !retracting) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        } else if (runtime.seconds() > 120 && !retracting) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        } */




        // Thread.currentThread().setPriority(Thread.MAX_PRIORITY);


        float Ch1 = gamepad1.right_stick_x;
        float Ch3 = gamepad1.left_stick_y;
        float Ch4 = gamepad1.left_stick_x;



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
            hange.setLiftPower(1);
        } else if (gamepad1.left_trigger > .25 && hanger) {
            hange.setLiftPower(-1);
        } else {
            hang.setPower(0);
        }





        if (gamepad1.right_bumper) {
            in.setPower(1);
        } else if (gamepad1.right_trigger > .25 && !retracting) {
            in.setPower(-1);
            intaking = true;

        }
        else if (gamepad1.right_trigger < .25 && !retracting) {
            in.setPower(0);
            intaking = false;
        }
        if (gamepad1.x) {

            dump.setPosition(dumpPos);
            dumped = true;
            isWaiting3 = false;


        } else if (gamepad1.b) {
            dump.setPosition(dumpIdle);
            dumped = false;
        }
        if (gamepad1.dpad_up) {
            button = false;
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
                button = true;
                tilt.setPosition(tiltUp);
            }
        }
        if (gamepad1.dpad_left) {
            g.setPosition(gClosed);
        }
        if (gamepad1.dpad_right) {
            button = true;
            g.setPosition(gOpen);
        }
        if (gamepad1.right_stick_button) {
            button = true;
            extending = true;
            hanger = false;
            slow = true;
            tilt.setPosition(tiltUp);
            //raise();
            g.setPosition(gClosed);

        }
        if (gamepad1.left_stick_button) {
            hanger = true;
            hange.setLiftSetpoint(-2870);
        }


        if (gamepad1.left_bumper && !hanger) {
            lowering = true;
            retracting = false;
            //  preExtend();
            dump.setPosition(dumpIdle);

            if (lifting) {
                tilt.setPosition(tiltUp);
                g.setPosition(gClosed);
                lifting = false;
            }
            if (!dumped) {
                lifter.setLiftSetpoint(0);
            }
            if (lift.getCurrentPosition() <= 10) {
                lifter.relinquish();
                lowering = false;
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








        double extendBack = 980 ;
        double inStopPos = 530;
        if (retracting && extend.getCurrentPosition() <= extendBack && !touched) {
            tilt.setPosition(tiltUp);
        } else if (retracting && extend.getCurrentPosition() > extendBack && !touched) {
            tilt.setPosition(tiltDown);
        }
        if (retracting) {
            dump.setPosition(.15);

        }
        if (retracting && extend.getCurrentPosition() <= 600) {
            g.setPosition(.5);
        }
        if (retracting && extend.getCurrentPosition() <= 400) {
            g.setPosition(gOpen);
        }

        lifter.update();
        hange.update();


        if (retracting && !touch.getState()) {
            g.setPosition(gOpen);
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        }
        if (extending && !retracting && !button){
            g.setPosition(gClosed);
        }
        if (extending && !retracting && extend.getCurrentPosition() > 600 && !button) {
            tilt.setPosition(tiltDown);
        }


        telemetry.addData("lift pos", lift.getCurrentPosition());
        telemetry.addData("hang pos", hang.getCurrentPosition());
        telemetry.addData("Right Omni", rightBackWheel.getCurrentPosition()/4);
        telemetry.addData("Left Omni", in.getCurrentPosition()/4);
        telemetry.addData("Horizontal Omni", rightFrontWheel.getCurrentPosition()/4);
        telemetry.addData("LB", leftBackWheel.getCurrentPosition());
        telemetry.addData("LF", leftFrontWheel.getCurrentPosition());
        telemetry.addData("extend", extend.getCurrentPosition());
        telemetry.addData("touched", touch.getState());
        telemetry.addData("Lift current", lift.getCurrentDraw()/1000d);
        telemetry.addData("RB current", rightBackWheel.getCurrentDraw()/1000d);
        telemetry.addData("LF current", leftFrontWheel.getCurrentDraw()/1000d);
        telemetry.addData("RF current", rightFrontWheel.getCurrentDraw()/1000d);
        telemetry.addData("LB current", leftBackWheel.getCurrentDraw()/1000d);
        telemetry.addData("Extend current", extend.getCurrentDraw()/1000d);
        telemetry.addData("in current",in.getCurrentDraw()/1000d);
        telemetry.addData("Hang current", hang.getCurrentDraw()/1000d);
        telemetry.addData("Total hub3 current",hub3.getTotalModuleCurrentDraw()/1000d);
        telemetry.addData("Total hub10 current",hub10.getTotalModuleCurrentDraw()/1000d);
        telemetry.addData("12v",hub3.read12vMonitor());
        telemetry.addData("old",old/1000d);


        double newV = in.getCurrentDraw();
        old = (newV * ratio) + ((1-ratio) * old);

        if (old >= 2400) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

    }


    @Override
    public void stop() {
        lifter.close();
        hange.close();
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
        dump.setPosition(.2);
        lifter.setLiftSetpoint(673);
        dumped = false;

    }

    public void retract() {

        if (retracting) {
            in.setPower(-1);
            if (!touch.getState() && retracting) {
                extend.setPower(0);
                touched = true;
                liftable = true;
                extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                in.setPower(-1);
                g.setPosition(gOpen);
                //tilt.setPosition(.7);

                if (!isWaiting)
                    waitTime = System.currentTimeMillis();
                isWaiting = true;
            }
            else if (i < 1) {
                extend.setPower(-1);

            }

        }
        double LowPos = .88;
        double UpPos = tiltUp;
        if (System.currentTimeMillis() - waitTime > 50 && isWaiting && !lifting && retracting) {
            tilt.setPosition(LowPos);
        }
        if (System.currentTimeMillis() - waitTime > 100 && isWaiting && !lifting && retracting) {
            tilt.setPosition(UpPos);
        }
        if (System.currentTimeMillis() - waitTime > 150 && isWaiting && !lifting && retracting) {
            tilt.setPosition(LowPos);
        }
        if (System.currentTimeMillis() - waitTime > 200 && isWaiting && !lifting && retracting) {
            tilt.setPosition(UpPos);
        } if (System.currentTimeMillis() - waitTime > 250 && isWaiting && !lifting && retracting) {
            tilt.setPosition(LowPos);
        }
        if (System.currentTimeMillis() - waitTime > 300 && isWaiting && !lifting && retracting) {
            tilt.setPosition(UpPos);
        }
        if (System.currentTimeMillis() - waitTime > 350 && isWaiting && !lifting && retracting) {
            tilt.setPosition(LowPos);
        }
        if (System.currentTimeMillis() - waitTime > 400 && isWaiting && !lifting && retracting) {
            tilt.setPosition(UpPos);
        }

        if (System.currentTimeMillis() - waitTime > 350 && isWaiting && !lifting && retracting) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            tilt.setPosition(UpPos);

            // tilt.setPosition(.9);
            in.setPower(0);
            if (liftable) {
                lift();
            }
            retracting = false;
            isWaiting = false;
            liftable = false;
            touched = false;
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
            dump.setPosition(dumpIdle);
            extend.setPower(1);
            if (!isWaiting2) {
                waitTime2 = System.currentTimeMillis();
                isWaiting2 = true;
            }
            if (System.currentTimeMillis() - waitTime2 > 250 && isWaiting2) {
                lifter.setLiftSetpoint(0);
                isWaiting2 = false;
                dumped = false;
                lifting = false;
                lowering = false;
                extend.setPower(0);

            }

        }
    }

}
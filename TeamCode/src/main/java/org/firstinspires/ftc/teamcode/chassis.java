package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="RR2", group="LANbros")
@Disabled
public class chassis extends OpMode {

    private ScoringLift lifter;
    private Extension extender;

    public chassis() {

    }


    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;

     DcMotor lift;
     DcMotor in;
     DcMotor extend;
   //  DcMotor lift;
     Servo dump;
     Servo g;
     Servo tilt;
     Servo wheel;
     DcMotor hang;
     boolean slow = false;
     boolean dumped = false;
     boolean extending = false;
     boolean lifting = false;
     boolean dumping;
    double gUpKp = 0.004;
    int gUpTolerance = 8;
    double gDownKp = 0.0018;
    int gDownTolerance = 4;
    double eUpKp = 0.004;
    int eUpTolerance = 8;
    double eDownKp = 0.0018;
    int eDownTolerance = 5;
    double dumpIdle = 0.17;
    double dumpPos = 0.3;
    boolean hanger = false;
    boolean retract = false;
    boolean retracting = false;
    int i = 0;

    double tiltUp = .444;
    double tiltDown = .755;

    double gClosed = .72;
    double gOpen = .33;
    @Override
    public void init() {


        leftFrontWheel = hardwareMap.dcMotor.get("LF");
        leftBackWheel = hardwareMap.dcMotor.get("LB");
        rightFrontWheel = hardwareMap.dcMotor.get("RF");
        rightBackWheel = hardwareMap.dcMotor.get("RB");
        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        in = hardwareMap.dcMotor.get("in");
        in.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend = hardwareMap.dcMotor.get("extend");
        tilt = hardwareMap.servo.get("tilt");
        g = hardwareMap.servo.get("g");
        dump = hardwareMap.servo.get("dump");
        hang = hardwareMap.dcMotor.get("hang");
        wheel = hardwareMap.servo.get("wheel");

        in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      // in.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter = new ScoringLift(lift, gUpKp, gUpTolerance, gDownKp, gDownTolerance);
        extender = new Extension(extend,eUpKp,eUpTolerance,eDownKp,eDownTolerance);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }


    public void start() {
        g.setPosition(gClosed);
        tilt.setPosition(tiltDown);
        dump.setPosition(dumpIdle);
        wheel.setPosition(.9);
        //extender.setLiftSetpoint(80);
    }


    @Override
    public void loop() {

        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float) scaleInput(right);
        left = (float) scaleInput(left);



        // write the values to the motors
        rightFrontWheel.setPower(right);
        rightBackWheel.setPower(right);
        leftBackWheel.setPower(left);
        leftFrontWheel.setPower(left);



        if (gamepad1.back) {
            hanger = true;
        } else if (gamepad1.start) {
            hanger = false;
        }

        if (gamepad1.left_bumper && hanger){
            hang.setPower(1);
        } else if (gamepad1.left_trigger >.25 && hanger){
                hang.setPower(-1);
            }
        else {
            hang.setPower(0);
        }

        if(hanger) {
            lifter.relinquish();
            extender.relinquish();
        }


        if (gamepad1.right_bumper && !hanger) {
            in.setPower(1);
        } else if (gamepad1.right_trigger > .25 && !slow && !hanger) {
            in.setPower(-1);
         } else if (gamepad1.right_trigger > .25 && slow && !hanger)
          {
             in.setPower(-1);
         } else {
            in.setPower(0);
        }

        if (gamepad1.x) {
            dump.setPosition(dumpPos);
            dumped = true;
        } else if (gamepad1.b) {
            dump.setPosition(dumpIdle);
        }
        if (gamepad1.dpad_up) {
            extending = true;
            retracting = false;
            extender.setLiftPower(1);
        } else if (gamepad1.dpad_down) {

            if (!retract || !retracting) {
                retract = true;
                retracting = true;
                i = 0;

            }
        }
        else if (!retracting || !retract) {
                extend.setPower(0);
            }
        retract();

        if (gamepad1.a) {
            if (extend.getCurrentPosition() > 1500) {
                tilt.setPosition(.77);
            }
            if (slow) {
                drop();
                tilt.setPosition(tiltDown);
            } else {
                drop();
                tilt.setPosition(tiltDown);
            }
        }

        if (gamepad1.y) {
            if (slow) {
                //dump.setPosition(1);
                tilt.setPosition(tiltUp);
            } else {
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
           slow = true;
               tilt.setPosition(tiltUp);
               raise();
               g.setPosition(gClosed);
           }
        if (gamepad1.left_stick_button) {
            slow = false;
            extending = false;
            g.setPosition(gOpen);
            tilt.setPosition(tiltDown);
        }


       if(gamepad1.left_bumper && !hanger) {

           if (dumped) {
               dump.setPosition(dumpIdle);
               dumped = false;
           }
           if (lifting) {
               tilt.setPosition(tiltUp);
               raise();
               g.setPosition(gClosed);
               extender.setLiftPower(1);
               sleep(100);
               lifting = false;
           }
            lifter.setLiftSetpoint(0);
        }else if(gamepad1.left_trigger > .25 && !hanger) {
            //dump.setPosition(.1);
           /*if(!lifting) {
               lifting = true;
               new Thread() {
                   @Override
                   public void run() {
                       lift();
                       lifting = false;
                   }
               }.start();
           } */
           lifting = true;
           lift();
        } else {
            lifter.relinquish();
        }




        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        telemetry.addData("lift pos",lift.getCurrentPosition());
        telemetry.addData("hang pos",hang.getCurrentPosition());
        telemetry.addData("RB", rightBackWheel.getCurrentPosition());
        telemetry.addData("RF", rightFrontWheel.getCurrentPosition());
        telemetry.addData("LB", leftBackWheel.getCurrentPosition());
        telemetry.addData("LF", leftFrontWheel.getCurrentPosition());
        telemetry.addData("extend", extend.getCurrentPosition());


    }

    @Override
    public void stop() {
        lifter.close();
        extender.close();
    }


    double scaleInput(double dVal) {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, .2,.21,.22,.23, 0.24,.25,.26,.27,.28,.29,
                0.30,.32, 0.36,.40, 0.43,.47, 0.50,.53,.55, 0.60,.63,.65,.69, 0.72,.74,.77,.8,.83, 0.85,.87,.9,.93,.96, 1.00, 1.00 };

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

        // return scaled value.
        return dScale;
    }
    public void lift(){
        tilt.setPosition(tiltDown);
        in.setPower(1);
        //sleep(00);
        lifter.setLiftSetpoint(800);
    }

    public void retract() {
       // retracting = true;

        if (retracting) {
            if (extend.getCurrentPosition() <= 80 && retracting) {
                g.setPosition(gOpen);
                in.setPower(0);
                extender.relinquish();
                in.setPower(-.8);
                sleep(100);
                in.setPower(0);
                retracting = false;
            } else if (i < 1){
                in.setPower(.8);
                sleep(150);
                in.setPower(0);
                tilt.setPosition(tiltUp);
                extender.setLiftSetpoint(-30);
                i++;
            }
        }
    }


    public void sleep(double time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void raise() {
        in.setPower(-1);
        sleep(80);
        in.setPower(0);
        //in.setPower(-1);
    }

    public void drop() {
        in.setPower(1);
       sleep(80);
       in.setPower(0);

    }


}
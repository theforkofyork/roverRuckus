package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


@TeleOp(name="TeleTest", group="LANbros")
@Disabled
public class newTele extends OpMode {

    private ScoringLift lifter;
    private Extension extender;

    public newTele() {

    }


    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;




    @Override
    public void init() {


        leftFrontWheel = hardwareMap.dcMotor.get("LF");
        leftBackWheel = hardwareMap.dcMotor.get("LB");
        rightFrontWheel = hardwareMap.dcMotor.get("RF");
        rightBackWheel = hardwareMap.dcMotor.get("RB");


        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("status", "loop test... waiting for start");
        telemetry.update();


    }


    public void start() {

    }


    @Override
    public void loop() {


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



    }


    @Override
    public void stop() {

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

        // return scaled value.
        return dScale;
    }



}
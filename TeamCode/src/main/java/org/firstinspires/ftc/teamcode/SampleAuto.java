/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.Func;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Sample", group="Sample")

public class SampleAuto extends LinearOpMode

{
    enum State {
        Lower,
        Turn,
        Detect,
        BackUp,
        Turn2,
        WallDrive,
        Depot,
        Return,
        AlignWithDepot,
        Stop,

    }
    LBHW robot = new LBHW();
    public PID turnPID = new PID();

    boolean detected = false;

    State state;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;



    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    GoldAlignDetector detector;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        webcamName = hardwareMap.get(WebcamName.class, "cam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters2 = new VuforiaLocalizer.Parameters();

        parameters2.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        parameters2.fillCameraMonitorViewParent = true;

        parameters2.cameraName = webcamName;

        vuforia = new Dogeforia(parameters2);
        vuforia.enableConvertFrameToBitmap();



        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.1;

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();
        telemetry.update();






      /* try {
            // axis remap
            byte AXIS_MAP_CONFIG_BYTE = 0b00011000; //swaps y-z, 0b00100001 is y-x, 0x6 is x-z
            byte AXIS_MAP_SIGN_BYTE = 0b000; //x, y, z

            //Need to be in CONFIG mode to write to registers
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            Thread.sleep(100); //Changing modes requires a delay before doing anything else

            //Write to the AXIS_MAP_CONFIG register
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);

            //Write to the AXIS_MAP_SIGN register
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);

            //Need to change back into the IMU mode to use the gyro
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

            Thread.sleep(100); //Changing modes again requires a delay
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } */
        state = State.Lower;

        waitForStart();


        while (opModeIsActive()) {

            switch (state) {

                case Lower: {
                    robot.tilt.setPosition(.768);
                    robot.hang.setPower(-1);
                    sleep(2000);
                    robot.hang.setPower(0);
                    state = State.Turn;
                }
                break;

                case Turn: {
                    powerDrive(.34);
                    sleep(360);
                    right(.3);
                    left(-.3);
                    sleep(600);
                    powerDrive(0);
                    robot.hang.setPower(1);
                    sleep(1500);
                    robot.hang.setPower(0);
                    //sleep(1000);
                    powerDrive(0);
                    sleep(300);
                    state = State.Detect;
                }
                break;

                case Detect: {
                    while (opModeIsActive() && !detected) {
                        if (!detector.getAligned() && !detected) {
                            right(-.05);
                            left(.05);
                        } else if (detector.getAligned()) {
                            right(0);
                            left(0);
                            sleep(400);
                            detected = true;
                            right(-.6);
                            left(-.6);
                            //  robot.in.setPower(1);
                            sleep(950);
                            powerDrive(0);
                            sleep(500);
                            robot.in.setPower(0);
                            powerDrive(-.5);
                            sleep(960);
                            powerDrive(0);
                            //robot.wheel.setPosition(.1);
                            state = State.Stop;
                        }
                    }
                }break;

                case Turn2: {
                    rotateDegrees(50);
                    sleep(500);
                    powerDrive(.5);
                    sleep(2000);
                    powerDrive(-.5);
                    sleep(500);
                    powerDrive(0);
                    state = State.AlignWithDepot;
                }break;

                case AlignWithDepot: {
                   // robot.tilt.setPosition(.66);
                    rotateDegrees(109);
                    sleep(200);
                    powerDrive(.7);
                    sleep(1500);
                    powerDrive(0);
                    robot.in.setPower(-.8);
                    sleep(1000);
                    robot.in.setPower(0);
                    sleep(300);
                    state = State.Return;
                }break;

                case Return: {
                    //rotateDegrees(127);
                    powerDrive(-.8);
                    sleep(1430);
                    powerDrive(0);
                    robot.wheel.setPosition(0);
                    state = State.Stop;
                }break;

                case Stop: {
                    right(0);
                    left(0);
                    stop();
                }break;

            }
        /*the (.5, 1, 0) one should go 35.26° (degrees), according to wolfram alpha
I solved for arctan(.5/(root(2)/2)) in degrees*/


        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void powerDrive(double power) {
        right(-power);
        left(-power);
    }

    public void right(double power) {
        robot.rightBackWheel.setPower(power);
        robot.rightFrontWheel.setPower(power);
    }
    public void left(double power) {
        robot.leftFrontWheel.setPower(power);
        robot.leftBackWheel.setPower(power);
    }

    public void rotateDegrees(int desiredDegrees) {
        // Sorry. You can't just spin around.
        desiredDegrees %= 360;

        if (2 >= Math.abs(desiredDegrees)) {
            return;
        }

        double power = 0.11;

        boolean quit = false;
        while(opModeIsActive() && !quit) {
            robot.encoders();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (angles.firstAngle < desiredDegrees) { // turning right, so heading should get smaller
                right(power);
                left(-power);
            } else { // turning left, so heading gets bigger.
                right(-power);
                left(power);
            }
            final float headingDiff = Math.abs(desiredDegrees - angles.firstAngle );

            telemetry.addData("Headings", String.format("Target", desiredDegrees, angles.firstAngle));
            telemetry.update();

            quit = headingDiff <= 2;

        }
        right(0);
        left(0);
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
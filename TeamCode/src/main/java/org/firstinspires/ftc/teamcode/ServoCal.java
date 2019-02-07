/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Serv Position Test", group="Test")
//@Disabled
public class ServoCal extends LinearOpMode {

    Servo dump;
    boolean flip = true;
    double leftPosition, rightPosition;


    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        dump = hardwareMap.servo.get("dump");

        // run until the end of the match (driver presses STOP)
        boolean pressed = false;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            if (gamepad1.dpad_up && !pressed) {
                rightPosition += .01;
                pressed = true;
            }  else if (gamepad1.dpad_down && !pressed) {
                rightPosition -= .01;
                pressed = true;
            }  else if (gamepad1.dpad_left && !pressed) {
                leftPosition += .01;
                pressed = true;
            }  else if (gamepad1.dpad_right && !pressed) {
                leftPosition -= .01;
                pressed = true;
            }  else { pressed = false; }
            if (gamepad1.a) {
                dump.setPosition(rightPosition);
            } else if (gamepad1.b) {
                dump.setPosition(leftPosition);
            }








            telemetry.addData("X Position",leftPosition);
            telemetry.addData("Y Position",rightPosition);

            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode;

import android.renderscript.Script;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Hang {
    private DcMotor motor1;
    private int liftSetpoint1 = 0, liftSetpoint2 = 0;
    private boolean hold;
    private double upKp, downKp;
    private int upTolerance, downTolerance;
    private boolean running = true;

    public Hang(DcMotor motor1, double upKp, int upTolerance, double downKp, int downTolerance) {
        this.motor1 = motor1;

        this.upKp = upKp;
        this.upTolerance = upTolerance;
        this.downKp = downKp;
        this.downTolerance = downTolerance;
    }

    public void update() {
        if(hold) {
            int encPos1 = getEncPos1();
            if(encPos1 < liftSetpoint1 - Hang.this.upTolerance) {
                set1((liftSetpoint1 - encPos1) * Hang.this.upKp);
            } else if(encPos1 > liftSetpoint1 + Hang.this.downTolerance) {
                set1((liftSetpoint1 - encPos1) * Hang.this.downKp);
            } else {
                set1(0);
            }
        }
    }

    private void set1(double power) {
        motor1.setPower(power);
    }



    public void setLiftPower(double power) {
        set1(power);
        hold = false;
        this.liftSetpoint1 = getEncPos1();
    }

    public void setLiftSetpoint(int setpoint) {
        this.liftSetpoint1 = setpoint;
        hold = true;
    }

    public void relinquish() {
        hold = false;
    }

    public int getEncPos1() {
        return this.motor1.getCurrentPosition();
    }


    public void close() {
        running = false;
    }
}
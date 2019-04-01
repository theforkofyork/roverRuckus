package org.firstinspires.ftc.teamcode.roadrunner.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.LBHW;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.RoadRunnerDriveBase;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous
public class SplineTestOpMode extends LinearOpMode {
    LBHW robot = new LBHW();
    boolean touched = false;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        SampleMecanumDriveBase drive = new RoadRunnerDriveBase(hardwareMap);
        Pose2d startpos = new Pose2d(16,16,0.785398);
        drive.setPoseEstimate(startpos);

        Trajectory teamMarker = drive.trajectoryBuilder()
                .splineTo(new Pose2d(-5, 58, 3.10669))
                .build();
        Trajectory teamMarkerReturn = drive.trajectoryBuilder()
                .reverse()
                .splineTo(startpos)
                .build();
        Trajectory Pit1 = drive.trajectoryBuilder()
                .reverse()
                .forward(17)
                .build();
        Trajectory Score1 = drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(11, 14, 0.610865))
                .build();
        Trajectory Pit2 = drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(25, 25, 0.785398))
                .build();
        Trajectory Score2 = drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(11, 14, 0.610865))
                .build();

        waitForStart();

        runPath(drive,teamMarker,dashboard);
        runPath(drive,teamMarkerReturn,dashboard);


    }
    public void teamMarkerDeploy(){
        robot.tilt.setPosition(robot.tiltDown);
        robot.extend.setPower(1);
        sleep(700);
        robot.marker.setPosition(.5);
        robot.extend.setPower(-1);
        sleep(700);
        robot.extend.setPower(0);
        robot.tilt.setPosition(robot.tiltUp);
    }
    public void retract() {
        while (opModeIsActive() &&!touched) {
            double extendBack = 900;
            double inStopPos = 530;
            if (robot.touch.getState()) {
                robot.in.setPower(-1);
                robot.extend.setPower(-1);
                robot.dump.setPosition(.22);

                if (robot.extend.getCurrentPosition() <= extendBack) {
                    robot.tilt.setPosition(robot.tiltUp);
                } else if (robot.extend.getCurrentPosition() > extendBack) {
                    robot.tilt.setPosition(robot.tiltDown);
                }
            }
            if (!robot.touch.getState()) {
                robot.g.setPosition(robot.gOpen);
                sleep(300);
                robot.in.setPower(0);
                robot.extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                touched = true;
            }
        }

    }

    public void liftScore() {
        robot.lift.setPower(1);
        sleep(800);
        robot.dump.setPosition(robot.dumpPos);
        sleep(700);
        robot.dump.setPosition(robot.dumpIdle);
        sleep(80);
        robot.lift.setPower(-.8);
        sleep(700);
        robot.lift.setPower(0);
    }
    public void runPath(SampleMecanumDriveBase drive, Trajectory trajectory, FtcDashboard dashboard)  {
        drive.followTrajectory(trajectory);
        while (!isStopRequested() && drive.isFollowingTrajectory()) {
            Pose2d currentPose = drive.getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", currentPose.getHeading());

            fieldOverlay.setStrokeWidth(4);
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);
            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

            dashboard.sendTelemetryPacket(packet);

            drive.update();
        }
    }


}

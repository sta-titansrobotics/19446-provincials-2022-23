/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Left extends LinearOpMode
{
    //INTRODUCE VARIABLES HERE
    DcMotor leftLift, rightLift, arm;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    PIDController controller;

    public static double p = 0.13, i = 0, d = 0.0001;
    public static double f = 0.2;

    final double tickstoDegree = 1.19;

    public enum LiftState {
        LIFT_START,
        LIFT_MOVE,
    }

    int liftTargetPos = 0;
    double lift_power = 0;
    boolean lift_start = false;

    int armTargetPos = 0;
    double arm_power = 0;
    boolean arm_start = false;

    int target = 20;

    LiftState liftState = LiftState.LIFT_START;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        //HARDWARE MAPPING HERE etc.
        // Other
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        // lift motors
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initialize position
        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo servoScissor = hardwareMap.get(Servo.class, "scissor");

        // Reverse left lift motor
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(p, i, d);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence common = drive.trajectorySequenceBuilder(startPose)

                /* Preload

                1.Start with cone in the back, move arm to front/dropping position while moving forward, after small delay extend the lift to high junction
                2. Drop the cone

                 */

                /* First cycle

                 1. Once cone is dropped, move arm to back/pickup position and start to lower lift while robot moving backwards
                 3. Pick up cone
                 4. This time lift goes up first, very small delay --> start moving backward towards junction
                 5. After small delay, arm starts to swing back to front/dropping position
                 */

                // preload

                .splineTo(new Vector2d(-35, -20), Math.toRadians(90))
                .splineTo(new Vector2d(-27, -8), Math.toRadians(65))

                .waitSeconds(0.5)

                // cycle 1
                .setReversed(true)
                .splineTo(new Vector2d(-40, -11.2), Math.toRadians(180))
                .splineTo(new Vector2d(-60, -11.2), Math.toRadians(180))
                .setReversed(false)
                .waitSeconds(0.5)

                .splineTo(new Vector2d(-40, -11.2), Math.toRadians(0))
                .splineTo(new Vector2d(-30, -11.2), Math.toRadians(58))
                .waitSeconds(0.5)

                // parking
                .setReversed(true)
                .splineTo(new Vector2d(-39, -11.5), Math.toRadians(180))
                .setReversed(false)

                .build();


        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(common.end())

                .lineToSplineHeading(new Pose2d(-57.5, -12, Math.toRadians(0)))
                .build();

        TrajectorySequence middleTraj = drive.trajectorySequenceBuilder(common.end())

                // park
                .lineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(0)))
                .build();

        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(common.end())

                // park
                .lineToSplineHeading(new Pose2d(-11.5, -12, Math.toRadians(0)))
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }





        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        // autonomous code here
        drive.followTrajectorySequence(common);

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            // left trajectory
            drive.followTrajectorySequence(leftTraj);


        } else if (tagOfInterest.id == MIDDLE) {
            // middle trajectory
            drive.followTrajectorySequence(middleTraj);

        } else {
            // right trajectory
            drive.followTrajectorySequence(rightTraj);

        }

        while (opModeIsActive()) {

            switch (liftState) {
                case LIFT_START:
                    if (lift_start) {
                        leftLift.setTargetPosition(liftTargetPos);
                        rightLift.setTargetPosition(liftTargetPos);

                        leftLift.setPower(lift_power);
                        rightLift.setPower(lift_power);

                        liftState = LiftState.LIFT_MOVE;
                    }
                    break;
                case LIFT_MOVE:
                    if (leftLift.getCurrentPosition() < leftLift.getTargetPosition() + 10 && leftLift.getCurrentPosition() > leftLift.getTargetPosition() - 10) {
                        leftLift.setPower(0);
                        rightLift.setPower(0);

                        lift_start = false;
                        liftState = LiftState.LIFT_START;
                    }
                    break;

                default:
                    liftState = LiftState.LIFT_START;
            }

            controller.setPID(p, i, d);
            int armPos = arm.getCurrentPosition();

            double pid = controller.calculate(armPos, target);

            double ff = Math.cos(Math.toRadians(target / tickstoDegree)) * f;

            double power = ff + pid;

            arm.setPower(power);

            telemetry.addData("Left Lift Power: ", leftLift.getPower());
            telemetry.addData("Right Lift Power: ", rightLift.getPower());
            telemetry.addData("Left Lift Encoder: ", leftLift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder: ", rightLift.getCurrentPosition());
            telemetry.addData("Scissor Intake Position: ", servoScissor.getPosition());
            telemetry.update();
        }

    }

    /**
     * Moves lift to certain height
     * @param power max power
     * @param ticks target position
     */
    public void moveLift(double power, int ticks) {
        liftTargetPos = ticks;
        lift_power = power;
        lift_start = true;
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
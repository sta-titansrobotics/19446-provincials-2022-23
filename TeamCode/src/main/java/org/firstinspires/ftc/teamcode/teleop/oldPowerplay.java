package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//blah blah blah

@TeleOp
public class oldPowerplay extends LinearOpMode {

    DcMotor leftLift, rightLift;

    boolean pRightBumper = false;
    boolean pLeftBumper = false;

    boolean pGA2Y = false;
    boolean pGA2A = false;
    boolean pGA2X = false;


    boolean pUP;
    boolean pDOWN;

    boolean scissorToggle = false;

    int armPosition = 0;



    public enum LimitState {
        LIMIT_ON,
        LIMIT_OFF,
    }

    LimitState limitState = LimitState.LIMIT_ON;

    @Override
    public void runOpMode() {


        // Movement Motors
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //Reverse left side motors
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Other
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        // Reverse right lift motor
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        // lift motors
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo servoScissor = hardwareMap.get(Servo.class, "scissor");

        double scissorPos;
        double MIN_POSITION = 0, MAX_POSITION = 1;

        ElapsedTime rpTimer = new ElapsedTime();

        waitForStart();

        // set initial positions
        scissorPos = 0.5;

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // forward, back
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!

            // strafing
            double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing

            // turning
            double rx = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);

            // lift

            // joystick up returns negative values, joystick down returns positive values

            double lift_power = -gamepad2.left_stick_y; // flip it

            if (gamepad2.left_stick_y < 0) {

                leftLift.setPower(lift_power);
                rightLift.setPower(lift_power);


            } else if (gamepad2.left_stick_y > 0) {

                leftLift.setPower(lift_power);
                rightLift.setPower(lift_power);


            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
            }

            // arm
            double arm_power = -gamepad2.right_stick_y; // flip it

            if (gamepad2.right_stick_y < 0) {

                arm.setPower(arm_power);


            } else if (gamepad2.right_stick_y > 0) {

                arm.setPower(arm_power);


            } else {
                arm.setPower(0);
            }

            // scissor intake
            boolean ga2A = gamepad2.a;
            if (ga2A && !pGA2A) {

                if (scissorPos > 0) {
                    scissorPos += 0.01;
                }

            }
            pGA2A = ga2A;

            boolean ga2Y = gamepad2.y;
            if (ga2Y && !pGA2Y) {
                if (scissorPos < 1) {
                    scissorPos -= 0.01;
                }

            }
            pGA2Y = ga2Y;


            // limiters
            boolean ga2X = gamepad2.x;
            switch (limitState) {
                case LIMIT_ON:

                    if (leftLift.getCurrentPosition() < 0) {
                        leftLift.setPower(0.5);
                    }

                    if (rightLift.getCurrentPosition() < 0) {
                        rightLift.setPower(0.5);
                    }

                    if (ga2X && !pGA2X) {
                        limitState = LimitState.LIMIT_OFF;
                    }
                    pGA2X = ga2X;

                    break;

                case LIMIT_OFF:

                    if (ga2X && !pGA2X) {
                        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        limitState = LimitState.LIMIT_ON;
                    }
                    pGA2X = ga2X;
                    break;

                default:
                    limitState = LimitState.LIMIT_ON;
                    
            }

            // set positions to arm and scissor
            arm.setTargetPosition(armPosition);
            servoScissor.setPosition(Range.clip(scissorPos, MIN_POSITION, MAX_POSITION));

            // add telemetry data
            telemetry.addData("Left Lift Power: ", leftLift.getPower());
            telemetry.addData("Right Lift Power: ", rightLift.getPower());
            telemetry.addData("Left Lift Encoder: ", leftLift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder: ", rightLift.getCurrentPosition());
            telemetry.addData("Scissor Intake Position: ", servoScissor.getPosition());
            telemetry.addData("Arm Power: ", arm.getPower());
            telemetry.addData("Arm Encoder: ", arm.getCurrentPosition());
            telemetry.addData("Limit State: ", limitState);

            telemetry.update();

        }

    }



}
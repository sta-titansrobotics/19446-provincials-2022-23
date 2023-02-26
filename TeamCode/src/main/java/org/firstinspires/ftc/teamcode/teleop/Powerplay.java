package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//blah blah blah
// not expanding 0.63
// expanded out 0.44
// bottom cone intake lift 1855

@TeleOp
public class Powerplay extends LinearOpMode {

    DcMotor leftLift, rightLift, arm;

    PIDController controller;

    public static double p = 0.13, i = 0, d = 0.0001;
    public static double f = 0.2;

    final double tickstoDegree = 1.19;

    boolean pGA2Y = false;
    boolean pGA2A = false;
    boolean pGA2X = false;

    boolean scissorToggle = false;

    double GROUNDJUNC = 1001, LOWJUNC = 3459, MIDJUNC = 5279, HIGHJUNC = 6574;

    boolean pPresetUP = false, pPresetDOWN= false;
    int currPreset = 0, switchVal = 0;


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
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        // lift motors
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo servoScissor = hardwareMap.get(Servo.class, "scissor");

        double scissorPos;
        double MIN_POSITION = 0, MAX_POSITION = 1;

        ElapsedTime rpTimer = new ElapsedTime();

        waitForStart();

        // set initial positions
        int target = 20;
        scissorPos = 0.63;

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

            motorFL.setPower(frontLeftPower*0.9);
            motorBL.setPower(backLeftPower*0.9);
            motorFR.setPower(frontRightPower*0.9);
            motorBR.setPower(backRightPower*0.9);

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

            // arm code
            if (gamepad2.right_stick_y < 0) {

                target += 2;


            } else if (gamepad2.right_stick_y > 0) {

                target -= 2;
            }

            // back negative power
            // forward positive
            controller.setPID(p, i, d);
            int armPos = arm.getCurrentPosition();

            double pid = controller.calculate(armPos, target);

            double ff = Math.cos(Math.toRadians(target / tickstoDegree)) * f;

            double power = ff + pid;

            arm.setPower(power);

            // scissor intake
            boolean ga2Y = gamepad2.y;
            if (ga2Y && !pGA2Y) {

                if (scissorPos == 0.63) {
                    scissorPos = 0.44;
                } else {
                    scissorPos = 0.63;
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

            boolean changedPreset = false;
            boolean presetUp = gamepad2.dpad_up;
            if (presetUp && !pPresetUP) {
                changedPreset = true;
                currPreset++;
                if(currPreset > 3) {
                    currPreset = 0;
                }
            }
            pPresetUP = presetUp;

            boolean presetDOWN = gamepad2.dpad_down;
            if (presetDOWN && !pPresetDOWN) {
                changedPreset = true;
                currPreset--;
                if(currPreset < 0) {
                    currPreset = 3;
                }
            }
            pPresetDOWN = presetDOWN;


            if(switchVal == -1) {

            } else if (changedPreset){
                switchVal = currPreset;
            } else {
                switchVal = -2;
            }

            switch (switchVal) {
                case 0:


                    leftLift.setTargetPosition((int) GROUNDJUNC);
                    rightLift.setTargetPosition((int) GROUNDJUNC);

                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftLift.setPower(1);
                    rightLift.setPower(1);

                    switchVal = -1;


                    break;
                case 1:

                    leftLift.setTargetPosition((int) LOWJUNC);
                    rightLift.setTargetPosition((int) LOWJUNC);

                    leftLift.setPower(1);
                    rightLift.setPower(1);

                    switchVal = -1;


                    break;
                case 2:

                    leftLift.setTargetPosition((int) MIDJUNC);
                    rightLift.setTargetPosition((int) MIDJUNC);

                    leftLift.setPower(1);
                    rightLift.setPower(1);

                    switchVal = -1;


                    break;
                case 3:

                    leftLift.setTargetPosition((int) HIGHJUNC);
                    rightLift.setTargetPosition((int) HIGHJUNC);

                    leftLift.setPower(1);
                    rightLift.setPower(1);

                    switchVal = -1;


                    break;
                case -1:
                    if (leftLift.getCurrentPosition() < leftLift.getTargetPosition() + 10 && leftLift.getCurrentPosition() > leftLift.getTargetPosition() - 10) {
                        leftLift.setPower(0);
                        rightLift.setPower(0);

                        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        changedPreset = false;
                        switchVal = -2;
                    }
                    break;
                case -2:
                    break;

            }

            // set positions to arm and scissor
            servoScissor.setPosition(Range.clip(scissorPos, MIN_POSITION, MAX_POSITION));

            // add telemetry data
            telemetry.addData("Left Lift Power: ", leftLift.getPower());
            telemetry.addData("Right Lift Power: ", rightLift.getPower());
            telemetry.addData("Left Lift Encoder: ", leftLift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder: ", rightLift.getCurrentPosition());
            telemetry.addData("Scissor Intake Position: ", servoScissor.getPosition());
            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.addData("Actual Arm Power: ", arm.getPower());
            telemetry.addData("power", power);
            telemetry.addData("Limit State: ", limitState);
            telemetry.update();

        }

    }



}
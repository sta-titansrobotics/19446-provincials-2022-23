package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class RevisedTeleOp extends LinearOpMode{

    // Movement Motors
    DcMotor motorFL, motorBL, motorFR, motorBR, leftLift, rightLift, Arm;

    Servo ScissorServo;

    boolean pGA2A = false;

    double Plateau = 600;

    double slowSpeed = 0.7;
    double strafeCounteract = 1.1;

    double scissorOpen = 0.67;
    double scissorClosed = 0.5;

    boolean scissorToggle = false;

    double armReturn;
    double armStartPos;
    boolean armModeToggle = false;
    boolean armMoveUp = false;

    double MAXPOSITION = 7100;


    double GROUNDJUNC = 0, LOWJUNC = 100, MIDJUNC = 500, HIGHJUNC = 1000;


    boolean pPresetUP = false, pPresetDOWN= false;
    int currPreset = 0, switchVal = 0;


    @Override
    public void runOpMode() {

        // Initialize Wheel Motors
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Lift Motors
        DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        // lift motors properties
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Arm Motor
        DcMotor Arm = hardwareMap.get(DcMotor.class, "arm");

        // arm motor properties
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Scissor Servo
        Servo ScissorServo = hardwareMap.get(Servo.class, "scissor");

        // scissor servo properties
        double scissorPos;
        double MIN_POSITION = 0, MAX_POSITION = 1;

        waitForStart();

        scissorPos = scissorClosed;

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            /* DRIVETRAIN */

            // forward, back
            double y = gamepad1.left_stick_y; // Remember, this is reversed!

            // strafing
            double x = -gamepad1.right_stick_x * strafeCounteract; // Counteract imperfect strafing

            // turning
            double rx = -gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);

            /* LIFT */
            // Apply power to Lift
            //Lift(liftPowerLeft(), liftPowerRight());
            double gamepad2Y = -gamepad2.left_stick_y;
            if(gamepad2Y != 0 && leftLift.getCurrentPosition() >= -100) {
                liftPowerLeft(leftLift, gamepad2Y);
                liftPowerRight(rightLift, gamepad2Y);
            }


            if (leftLift.getCurrentPosition() < 0){
                leftLift.setPower(0.5);
            }
            if (leftLift.getCurrentPosition() > MAXPOSITION){
                leftLift.setPower(-0.5);
            }

            if (rightLift.getCurrentPosition() < 0){
                rightLift.setPower(0.5);
            }
            if (rightLift.getCurrentPosition() > MAXPOSITION){
                rightLift.setPower(-0.5);
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

            /* ARM */
            // Apply power to Arm
            //Arm.setPower(armPower());
            Arm.setPower(-gamepad2.right_stick_y);

            /* SCISSOR */
            // scissor intake
            boolean ga2A = gamepad2.a;

            // toggle scissor
            if (ga2A && !pGA2A) {
                scissorToggle = !scissorToggle;
            }

            // pick up (expand scissor)
            if (scissorToggle) {
                scissorPos = scissorOpen;
            }

            // release cone (neutral position)
            else {
                scissorPos = scissorClosed;
            }

            // untoggle scissor
            pGA2A = ga2A;

            // Apply power to Scissor
            ScissorServo.setPosition(Range.clip(scissorPos, MIN_POSITION, MAX_POSITION));


            /* TELEMETRY OUTPUT */
            // motor data
            telemetry.addLine("Wheel Data");
            telemetry.addData("Front Left Motor Busy: ", motorFL.isBusy() + " Power: " + frontLeftPower);
            telemetry.addData("Front Right Motor Busy: ", motorFR.isBusy() + " Power: " + frontRightPower);
            telemetry.addData("Front Left Motor Busy: ", motorBL.isBusy() + " Power: " + backLeftPower);
            telemetry.addData("Front Right Motor Busy: ", motorBR.isBusy() + " Power: " + backRightPower);
            telemetry.addData("Gamepad Left Y: ", gamepad1.left_stick_y + " Gamepad Left X: " + gamepad1.left_stick_x);
            telemetry.addData("Gamepad Right Y: ", gamepad1.right_stick_y + " Gamepad Right X: " + gamepad1.right_stick_x);

            // scissor data
            telemetry.addLine("");
            telemetry.addLine("Scissor Data:");
            telemetry.addData("Scissor Toggle: ", scissorToggle + " " + scissorPos);

            // arm data
            telemetry.addLine("");
            telemetry.addLine("Arm Data:");
            telemetry.addData("Arm Power: ", Arm.getPower() + " Pos" + Arm.getCurrentPosition());

            // lift data
            telemetry.addLine("");
            telemetry.addLine("Lift Data:");
            telemetry.addData("Left Lift Power: ", leftLift.getPower() + " Right Lift Power: " + rightLift.getPower());
            telemetry.addData("Lift Currrent Pos: ", leftLift.getCurrentPosition() + " "+ rightLift.getCurrentPosition());

            // update output
            telemetry.update();
        }
    }

    /**
     * @author Gregory L, Anujan K, Andrew C
     * @param lLift Left lift motor being controlled by this method.
     * calculates and returns required power to the left lift.
     */
    public void liftPowerLeft(DcMotor lLift, double setPower) {
        if (lLift.getCurrentPosition() > 0){
            lLift.setPower(setPower * 0.85);
        }

        if(lLift.getCurrentPosition() < MAXPOSITION) {
            lLift.setPower(setPower * 0.35);
        }
    }

    /**
     * @author Gregory L, Anujan K, Andrew C
     * @param rLift  Right lift motor being controlled by this method.
     * calculates and returns required power to the right lift.
     */
    public void liftPowerRight(DcMotor rLift, double setPower){
        if (rLift.getCurrentPosition() > 0){
            rLift.setPower(setPower * 0.85);
        }

        if(rLift.getCurrentPosition() < MAXPOSITION) {
            rLift.setPower(setPower * 0.35);
        }
    }

    /**
     * @author Gregory L
     * @return Power for the arm
     * calculates and returns required power to the arm.
     */
    public void armPower(){

    }







    }


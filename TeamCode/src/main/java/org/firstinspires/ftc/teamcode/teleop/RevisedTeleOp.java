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



    double strafeCounteract = 1.1;

    double scissorOpen = 0.67;
    double scissorClosed = 0.5;

    boolean scissorToggle = false;

    double LIFTMAXPOSITION = 10000000;

    double GROUNDJUNC = 0, LOWJUNC = 100, MIDJUNC = 500, HIGHJUNC = 1000;

    boolean pPresetUP = false;
    int liftCurrPreset = 0;

    double ARMMAXPOSITION = 1000000;

    double DOWNPOS = 0, FORWARDPOS = 100, BACKPOS = 300, PEAK = 200;
    double armPower = 0;

    int armCurrPreset = 0;

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
            double gamepad2Y = -gamepad2.left_stick_y;
            if(gamepad2Y != 0) {
                liftPowerLeft(leftLift, -gamepad2Y);
                liftPowerRight(rightLift, -gamepad2Y);
            }

            int liftReturnTo = 0;

            switch (liftCurrPreset) {
                case 0:

                    if(gamepad2.dpad_up) {

                        liftReturnTo = 1;

                        leftLift.setTargetPosition((int) GROUNDJUNC);
                        rightLift.setTargetPosition((int) GROUNDJUNC);

                        leftLift.setPower(1);
                        rightLift.setPower(1);

                        liftCurrPreset = -1;


                    } else if (gamepad2.dpad_down){
                        liftReturnTo = 3;

                        leftLift.setTargetPosition((int) GROUNDJUNC);
                        rightLift.setTargetPosition((int) GROUNDJUNC);

                        leftLift.setPower(1);
                        rightLift.setPower(1);

                        liftCurrPreset = -1;
                    }
                    break;
                case 1:

                    if(gamepad2.dpad_up) {

                        liftReturnTo = 2;

                        leftLift.setTargetPosition((int) LOWJUNC);
                        rightLift.setTargetPosition((int) LOWJUNC);

                        leftLift.setPower(1);
                        rightLift.setPower(1);

                        liftCurrPreset = -1;


                    } else if (gamepad2.dpad_down){
                        liftReturnTo = 0;

                        leftLift.setTargetPosition((int) LOWJUNC);
                        rightLift.setTargetPosition((int) LOWJUNC);

                        leftLift.setPower(1);
                        rightLift.setPower(1);

                        liftCurrPreset = -1;
                    }
                    break;
                case 2:

                    if(gamepad2.dpad_up) {

                        liftReturnTo = 3;

                        leftLift.setTargetPosition((int) MIDJUNC);
                        rightLift.setTargetPosition((int) MIDJUNC);

                        leftLift.setPower(1);
                        rightLift.setPower(1);

                        liftCurrPreset = -1;


                    } else if (gamepad2.dpad_down){
                        liftReturnTo = 1;

                        leftLift.setTargetPosition((int) MIDJUNC);
                        rightLift.setTargetPosition((int) MIDJUNC);

                        leftLift.setPower(1);
                        rightLift.setPower(1);

                        liftCurrPreset = -1;
                    }
                    break;
                case 3:

                    if(gamepad2.dpad_up) {

                        liftReturnTo = 0;

                        leftLift.setTargetPosition((int) HIGHJUNC);
                        rightLift.setTargetPosition((int) HIGHJUNC);

                        leftLift.setPower(1);
                        rightLift.setPower(1);

                        liftCurrPreset = -1;


                    } else if (gamepad2.dpad_down){
                        liftReturnTo = 2;

                        leftLift.setTargetPosition((int) HIGHJUNC);
                        rightLift.setTargetPosition((int) HIGHJUNC);

                        leftLift.setPower(1);
                        rightLift.setPower(1);

                        liftCurrPreset = -1;
                    }
                    break;
                case -1:
                    if (leftLift.getCurrentPosition() < leftLift.getTargetPosition() + 10 && leftLift.getCurrentPosition() > leftLift.getTargetPosition() - 10) {
                        leftLift.setPower(0);
                        rightLift.setPower(0);

                        liftCurrPreset = liftReturnTo;
                    }
                    break;


            }

            /* ARM */
            // Apply power to Arm

            if (gamepad2.right_stick_y != 0 && !Arm.isBusy()){
                armMovement(Arm, -gamepad2.right_stick_y);
            }


            if (gamepad2.x && !Arm.isBusy()){
                Arm.setTargetPosition((int) DOWNPOS);
            }
            if (gamepad2.y && !Arm.isBusy()){
                Arm.setTargetPosition((int) FORWARDPOS);
            }
            if (gamepad2.b && !Arm.isBusy()){
                Arm.setTargetPosition((int) BACKPOS);
            }

            if (Arm.getCurrentPosition() > Arm.getTargetPosition()){
                Arm.setPower(-1);
            } else if (Arm.getCurrentPosition() < Arm.getCurrentPosition()){
                Arm.setPower(1);
            } else if (Arm.getCurrentPosition() < Arm.getCurrentPosition() + 10 && Arm.getCurrentPosition() > Arm.getCurrentPosition() - 10){
                Arm.setPower(0);
            }

            /* SCISSOR */
            // scissor intake
            boolean ga2A = gamepad2.a;

            // toggle scissor
            if (ga2A && !gamepad2.a) {
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
            telemetry.addData("Arm Power: ", Arm.getPower());

            // lift data
            telemetry.addLine("");
            telemetry.addLine("Lift Data:");
            telemetry.addData("Left Lift Power: ", leftLift.getPower() + " Right Lift Power: " + rightLift.getPower());

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

        if(lLift.getCurrentPosition() < LIFTMAXPOSITION) {
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

        if(rLift.getCurrentPosition() < LIFTMAXPOSITION) {
            rLift.setPower(setPower * 0.35);
        }
    }

    /**
     * @author Gregory L
     * @param armMotor Arm motor being controlled by this method.
     * calculates and returns required power to the arm.
     */
    public void armMovement(DcMotor armMotor, double setPower){

        if (armMotor.getCurrentPosition() > 0){
            armMotor.setPower(setPower * 0.85);
        }

        if (armMotor.getCurrentPosition() < ARMMAXPOSITION) {
            armMotor.setPower(setPower * 0.35);
        }
    }
}

package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Single Motor Test", group = "test")

public class SingleMotorTest extends LinearOpMode {

    private DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException{

        motor = hardwareMap.get(DcMotorEx.class, "motor");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean velocity = false;

        while(!opModeIsActive() && !isStopRequested()) {

            sleep(200);

            if(gamepad1.a && motor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            else if(gamepad1.a)
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(gamepad1.x && !velocity){
                velocity = true;
            }
            else if(gamepad1.x && velocity){
                velocity = false;
            }

            telemetry.addData("Set run mode with ", "button A on gamepad1");
            telemetry.addData("Current run mode ", motor.getMode());
            telemetry.addData("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", "");
            telemetry.addData("Change to velocity or power with ", "button X on gamepad1");

            if(velocity)
                telemetry.addData("Current speed setting mode ", "velocity");
            else telemetry.addData("Current speed setting mode ", "power");

            telemetry.addData("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", "");
            telemetry.addData("Waiting for start", "");
            telemetry.update();
        }

        boolean isPressed = false;
        double motorPower = 0;
        double motorVelocity = 0;

        while(opModeIsActive()){

            if(gamepad1.y && !isPressed) {

                isPressed = true;

                if(!velocity && motorPower < 1 )
                    motorPower += 0.05;
                else if(velocity)
                    motorVelocity += 50;


            }
            else if (gamepad1.a && !isPressed){

                isPressed = true;

                if(motorPower > 0 && !velocity)
                    motorPower -= 0.05;
                else if(velocity && motorVelocity > 0)
                    motorVelocity -= 50;
            }
            else if(!gamepad1.a && !gamepad1.y)
                isPressed = false;

            if(gamepad1.dpad_up) {
                if (!velocity)
                    motor.setPower(motorPower);
                else motor.setVelocity(motorVelocity);
            }
            else if(gamepad1.dpad_down) {
                if(!velocity)
                    motor.setPower(-motorPower);
                else motor.setVelocity(-motorVelocity);
            }
            else {
                if(!velocity)
                    motor.setPower(0);
                else motor.setVelocity(0);
            }

            telemetry.addData("Motor run mode: ", motor.getMode());
            telemetry.addData("Motor position: ", motor.getCurrentPosition());
            telemetry.addData("~~~~~~~~~~~~~~~~~~~~~~~", "");
            telemetry.addData("Set power: ", motorPower);
            telemetry.addData("Actual power: ", motor.getPower());
            telemetry.addData("~~~~~~~~~~~~~~~~~~~~~~~", "");
            telemetry.addData("Set velocity ", motorVelocity);
            telemetry.addData("Actual velocity (ticks per second)", motor.getVelocity());
            telemetry.update();
        }
    }
}

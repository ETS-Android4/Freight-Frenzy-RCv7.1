package org.firstinspires.ftc.teamcode.Main;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.commands.core.LynxReadVersionStringResponse;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


//Road Runner Imports - Lucian

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.Autonomie.DetectObject;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.nio.file.attribute.FileOwnerAttributeView;
import java.util.function.Function;

@TeleOp(name = "Main_Driving", group = "main")

public class Driving1 extends LinearOpMode {

    //Motoare Brat, Slidere, Carusel, Colector
    private DcMotorEx motor_slider;
    private DcMotorEx motor_carusel;
    private DcMotorEx motor_colector;
    private DcMotorEx motor_turela;
    //private ColorSensor csensor1;
    //Valoare Pentru Inversare Orientare Driving

    SampleMecanumDrive mecanum_drive;

    //Variabila Dashboard
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private CRServo holder;
    private CRServo trap;
    private CRServo stick;
    private Servo cup;
    private CRServo OTcup;
    private CRServo OTcup2;

    private DistanceSensor dsensor1;
    //Variabila cu clasa de functii
    private functions fx = new functions();

    OpenCvWebcam webcam;
    DetectObject pipe_line = new DetectObject();
    @Override
    public void runOpMode() throws InterruptedException
    {
        //Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.setPipeline(pipe_line);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });



        //Init pentru motor miscare de extindere si retragere
        motor_slider = hardwareMap.get(DcMotorEx.class, "slider");

        motor_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_slider.setDirection(DcMotorSimple.Direction.FORWARD);

        //Init pentru motor miscare turela
        motor_turela = hardwareMap.get(DcMotorEx.class, "turela");

        motor_turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_turela.setDirection(DcMotorSimple.Direction.REVERSE);

        //Init pentru motorul are se cupa de aruselul cu rate
        motor_carusel = hardwareMap.get(DcMotorEx.class, "carusel");

        motor_carusel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_carusel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_carusel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_carusel.setDirection(DcMotorSimple.Direction.FORWARD);

        //Init pentru motorul care se ocupa cu colectarea elementelor
        motor_colector = hardwareMap.get(DcMotorEx.class, "colector");

        motor_colector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_colector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_colector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_colector.setDirection(DcMotorSimple.Direction.REVERSE);


        //Init pentru servo-ul de la holder de elemente
        holder = hardwareMap.crservo.get("holder");
        holder.setDirection(CRServo.Direction.REVERSE);

        trap = hardwareMap.crservo.get("trap");
        trap.setDirection(DcMotorSimple.Direction.REVERSE);

        stick = hardwareMap.crservo.get("stick");
        stick.setDirection(DcMotorSimple.Direction.REVERSE);

        cup = hardwareMap.servo.get("cup");
        cup.setPosition(0.85);

        //Batul de la cupa
        //am un cui si un pahar
        OTcup = hardwareMap.crservo.get("otcup");
        OTcup.setDirection(DcMotorSimple.Direction.FORWARD);
        // 1 - start, -0.25 max
        OTcup.setPower(1);

        //0 start + sus - jos
        //Cupa pentru cub
        OTcup2 = hardwareMap.crservo.get("otcup2");
        OTcup2.setPower(0.08);

        holder.setPower(-0.22);
        trap.setPower(1);
        stick.setPower(0.9);





        //Init pentru mecanum drive RR
        mecanum_drive = new SampleMecanumDrive(hardwareMap);
        mecanum_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanum_drive.setPoseEstimate(new Pose2d(0, 0, 0));

         dsensor1 = hardwareMap.get(DistanceSensor.class, "dsensor1");

        //Before start
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Apasa pe buton!", "");
            telemetry.update();
        }

        ElapsedTime timer = new ElapsedTime();
        boolean last_it_orientation = false;
        boolean curr_it_orientation = false;
        boolean last_it_speed = false;
        boolean curr_it_speed = false;
        boolean tgl_speed = false;
        boolean tgl_orientation = false;
        int orientation_drive = 1;
        int lower_ts = 3500;
        double speed_robot;
        //Intrare in program
        while (opModeIsActive())
        {
            //Schimbare orientare robot
            last_it_orientation = curr_it_orientation;
            curr_it_orientation = gamepad2.x;

            last_it_speed = curr_it_speed;
            curr_it_speed = gamepad2.left_bumper;

            if (curr_it_speed && !last_it_speed)
                tgl_speed = !tgl_speed;

            if (curr_it_orientation && !last_it_orientation)
                orientation_drive *= -1;

            if (tgl_speed)
                speed_robot = 0.3;
            else
                speed_robot = 0.6;


            telemetry.addData("Time: ", timer.seconds());
            if (timer.seconds() > 6)
                timer.reset();

            //Setare driving pe controller
            mecanum_drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad2.left_stick_y * orientation_drive * speed_robot,
                            -gamepad2.left_stick_x * orientation_drive * speed_robot,
                            -gamepad2.right_stick_x * speed_robot
                    )
            );
            mecanum_drive.update();


            //Functiile de baza
            fx.slider(true);
            fx.carusel(true);
            fx.colector();
            fx.reset_brat();
            //fx.stick_cr();
            fx.cup();
            fx.turela(true);
            fx.outtakep1();
            telemetry.addData("Speed: ", speed_robot);
            telemetry.update();
        }

    }


    //Clasa functii de baza robot
    class functions {
        //deprecated
        private int button_sleep = 135;

        //Valoare care retine daca bratul este in reset sau nu
        private boolean resetting_brat = false;

        //Limita de jos si de sus a bratului ca inclinare
        private final int upper_limit_brat = 1400;
        private final int lower_limit_brat = -4200;

        //Valoare care retine daca bratul este destul de extins pentru inclinare si distanta de clear
        private boolean clear_brat = false;
        private int dist_min_ext = 900;

        private double put_brat = 0.8;


        //Functia de resetare brat si slider
        boolean last_it_reset = false;
        boolean curr_it_reset = false;
        public void reset_brat() {
            last_it_reset = curr_it_reset;
            curr_it_reset = gamepad1.b;

            if (curr_it_reset && !last_it_reset) {
                resetting_brat = true;
                motor_slider.setTargetPosition(0);

                motor_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (Math.abs(motor_slider.getCurrentPosition()) >= 2)
                    motor_slider.setPower(1);
            }

            if (Math.abs(motor_slider.getCurrentPosition()) < 8)
                resetting_brat = false;

            if (resetting_brat == false) {
                motor_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        }

        private int left_limit_turela = -2500;
        private int right_limit_turela = 2500;
        private int poz_turela = 0;
        private double put_turela = 1;
        private int dif = 0;
        public void turela(boolean logs)
        {
            poz_turela = motor_turela.getCurrentPosition() * -1;
            if (gamepad1.left_stick_x >= 0.3 && poz_turela <= right_limit_turela)
                motor_turela.setPower(0.4);
            else if (gamepad1.left_stick_x <= -0.3 && poz_turela >= left_limit_turela)
                motor_turela.setPower(-0.4);
            else
                motor_turela.setPower(0);

            if (logs) {
                telemetry.addData("turela jx: ", gamepad1.left_stick_x);
                telemetry.addData("turela jy: ", gamepad1.left_stick_y);
                telemetry.addData("turela poz: ", poz_turela);
                telemetry.addData("turela pow: ", motor_turela.getPower());
            }
        }


        //Pozitie si limite slidere de extindere si retragere
        private int poz_slider = 0;
        private int upper_limit_slider = 1600;
        private int lower_limit_slider = 2;
        private double put_slider = 1;

        //Functia pentru motoarele de la slider extindere/retragere
        public void slider(boolean logs) {
            poz_slider = motor_slider.getCurrentPosition();

            if (resetting_brat == false) {
                if (gamepad1.right_trigger >= 0.3 && poz_slider <= upper_limit_slider) {
                    motor_slider.setPower(put_slider);
                } else if (gamepad1.left_trigger >= 0.3 && poz_slider >= lower_limit_slider) {
                    motor_slider.setPower(-put_slider);
                } else
                    motor_slider.setPower(0);
            }

            if (logs) {
                telemetry.addData("Pozitie Curenta Motor Slider:", poz_slider);
                telemetry.addData("Putere Teoretica Motor Slider:", put_slider);
                telemetry.addData("Putere Practica Motor Slider:", motor_slider.getPower());
            }

        }


        //Motor carusel si variabila toggle
        private boolean carusel_tgl_red = false;
        private boolean carusel_last_it_red = false;
        private boolean carusel_curr_it_red = false;
        private boolean carusel_tgl_blue = false;
        private boolean carusel_last_it_blue = false;
        private boolean carusel_curr_it_blue = false;
        double carusel_power = 0;
        public void carusel(boolean logs) {

            carusel_last_it_red = carusel_curr_it_red;
            carusel_curr_it_red = gamepad1.y;
            carusel_last_it_blue = carusel_curr_it_blue;
            carusel_curr_it_blue = gamepad1.x;

            if (carusel_curr_it_red && !carusel_last_it_red)
                carusel_tgl_red = !carusel_tgl_red;

            if (carusel_curr_it_blue && !carusel_last_it_blue)
                carusel_tgl_blue = !carusel_tgl_blue;

            if (carusel_tgl_red && !carusel_tgl_blue)
            {
                if (carusel_power > -0.60)
                    carusel_power -= 0.04;

            }
            else if (carusel_tgl_blue && !carusel_tgl_red)
            {
                if (carusel_power < 0.60)
                    carusel_power += 0.04;
            }
            else
                carusel_power = 0;

            motor_carusel.setPower(carusel_power);

            if (logs)
            {
                telemetry.addData("Toggle Red: ", carusel_tgl_red);
                telemetry.addData("Toggle Blue: ", carusel_tgl_blue);
                telemetry.addData("Carusel Power: ", carusel_power);

            }

        }

        //Functie pentru colector
        private int clear_colector = 200;
        boolean last_it_collector = false;
        boolean curr_it_collector = false;
        boolean tgl_collector = false;
        public void colector() {
            last_it_collector = curr_it_collector;
            curr_it_collector = gamepad2.y;
            if (curr_it_collector && !last_it_collector)
                tgl_collector = !tgl_collector;

            if (tgl_collector && gamepad2.right_bumper)
                motor_colector.setPower(-1);
            else if (tgl_collector && !gamepad2.right_bumper)
                motor_colector.setPower(0.55);
            else
                motor_colector.setPower(0);

        }

        /*public void stick_cr()
        {
            if (gamepad1.dpad_up && stick.getPower() <= 0.9)
                stick.setPower(stick.getPower() + 0.02);
            else if (gamepad1.dpad_down && stick.getPower() >= -0.84)
                stick.setPower(stick.getPower() - 0.02);



            telemetry.addData("Stick power:", stick.getPower());
        }*/

        double pow_cup = 0;
        double dist_sens;
        ElapsedTime cup_t = new ElapsedTime();
        double last_ct = 0;
        public void cup()
        {

            dist_sens = dsensor1.getDistance(DistanceUnit.MM);
            if (dist_sens <= 50)
            {
                if (last_ct == 0) {
                    cup_t.reset();
                    last_ct = cup_t.milliseconds();
                    tgl_collector = false;
                }
            }

            if (dist_sens > 70) {
                last_ct = 0;
            }

            if (dist_sens <= 50 && cup_t.milliseconds() > last_ct + 250)
            {
                last_ct = 0;
                cup_t.reset();
                cup.setPosition(0.4);
                tgl_collector = true;
            }

            if (dist_sens > 70 && cup_t.milliseconds() > last_ct + 700)
            {
                last_ct = 0;
                cup_t.reset();
                cup.setPosition(0.85);
            }

            telemetry.addData("Cup Power: ", pow_cup);
            telemetry.addData("DSensor: ", dist_sens);
        }

        boolean front_last = false;
        boolean front_curr = false;
        boolean cup_last = false;
        boolean cup_curr = false;
        boolean tgl_cup = false;
        boolean tgl_front = false;
        double otpow1 = 1;
        double otpow2 = 0;
        double l_time = 0;
        ElapsedTime cupt = new ElapsedTime();
        public void outtakep1()
        {

            front_last = front_curr;
            front_curr = gamepad1.right_bumper;
            cup_last = cup_curr;
            cup_curr = gamepad1.left_bumper;

            if (cup_curr & !cup_last)
                tgl_cup = !tgl_cup;

            if (front_curr && !front_last) {
                tgl_front = !tgl_front;
                cupt.reset();
                l_time = cupt.milliseconds();
            }

            if (tgl_front) {
                otpow1 = -0.10;
                otpow2 = -0.65;
            }

            if (tgl_cup)
            {
                otpow2 = 0.5;
            }

            if (!tgl_front)
            {
                otpow1 = 1;
                otpow2 = 0.08;
            }



        OTcup.setPower(otpow1);
        if (tgl_cup && tgl_front)
            OTcup2.setPower(otpow2);
        else if (cupt.milliseconds() > 275 + l_time) {
            OTcup2.setPower(otpow2);
            cupt.reset();
        }


        telemetry.addData("outtake", "");
        telemetry.addData("Timer cup: ", cupt.milliseconds());
        telemetry.addData("Front_toggle: ", tgl_front);
        telemetry.addData("Cup_toggle: ", tgl_cup);
        telemetry.addData("otpow1: ", otpow1);
        telemetry.addData("otpow2: ", otpow2);
        telemetry.addData("otpow_acc1: ", OTcup.getPower());
        telemetry.addData("otpow_acc2: ", OTcup2.getPower());
        telemetry.addData("outtake", "");
        }
    }
}
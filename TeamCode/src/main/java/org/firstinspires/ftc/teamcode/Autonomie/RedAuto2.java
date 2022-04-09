package org.firstinspires.ftc.teamcode.Autonomie;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "OPERATION: Carousel||Red", group = "main")
public class RedAuto2 extends LinearOpMode{

    //Motoare Brat, Slidere, Carusel, Colector
    private DcMotorEx motor_brat;
    private DcMotorEx motor_slider;
    private DcMotorEx motor_carusel;
    private DcMotorEx motor_colector;
    private DcMotorEx motor_turela;

    //Variabila Dashboard
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private SampleMecanumDrive mecanum_drive;

    private CRServo holder;
    private CRServo trap;
    private CRServo stick;
    private Servo cup;
    private CRServo OTcup;
    private CRServo OTcup2;
    private DistanceSensor dsensor1;
    private functions fx = new functions();

    private SampleMecanumDrive drive;

    OpenCvWebcam webcam;
    DetectObjectBlue pipe_line = new DetectObjectBlue();

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
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        //Init pentru motor miscare de extindere si retragere
        motor_slider = hardwareMap.get(DcMotorEx.class, "slider");

        motor_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_slider.setTargetPosition(0);
        motor_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_slider.setDirection(DcMotorSimple.Direction.FORWARD);

        //Init pentru motorul care se ocupa de caruselul cu rate
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

        //init motor turela
        motor_turela = hardwareMap.get(DcMotorEx.class, "turela");

        motor_turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_turela.setDirection(DcMotorSimple.Direction.REVERSE);


        //Init pentru servo-ul de la holder de elemente
        holder = hardwareMap.crservo.get("holder");
        holder.setDirection(CRServo.Direction.REVERSE);

        trap = hardwareMap.crservo.get("trap");
        trap.setDirection(DcMotorSimple.Direction.REVERSE);

        stick = hardwareMap.crservo.get("stick");
        stick.setDirection(DcMotorSimple.Direction.REVERSE);

        cup = hardwareMap.servo.get("cup");
        cup.setPosition(0.1);

        //Batul de la cupa
        //am un cui si un pahar
        OTcup = hardwareMap.crservo.get("otcup");
        OTcup.setDirection(DcMotorSimple.Direction.FORWARD);
        // 1 - start, -0.25 max
        OTcup.setPower(1);

        //0 start + sus - jos
        //Cupa pentru cub
        OTcup2 = hardwareMap.crservo.get("otcup2");
        OTcup2.setPower(0.65);

        holder.setPower(-0.22);
        trap.setPower(1);
        stick.setPower(1);

        //Init pentru mecanum drive RR
        mecanum_drive = new SampleMecanumDrive(hardwareMap);

        mecanum_drive.setPoseEstimate(new Pose2d(0, 0, 0));
        double y = -4;
        double x = -1;
        dsensor1 = hardwareMap.get(DistanceSensor.class, "dsensor1");
        int tip_autonomie;
        waitForStart();
        double last_x;
        double last_y;
        ElapsedTime timer_autonomie = new ElapsedTime();
        tip_autonomie = pipe_line.gen_tip_autonomie();

        if (isStopRequested()) return;

        drive = new SampleMecanumDrive(hardwareMap);
        ArrayList<Trajectory> path = new ArrayList<>();
        boolean start = false;

        telemetry.addData("Autonomie: ", tip_autonomie);
        telemetry.update();


        stick.setPower(0);
        cup.setPosition(0.3);
        OTcup2.setPower(0.65);
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0))
                .strafeLeft(6,
                        SampleMecanumDrive.getVelocityConstraint(30, 0.5, 10.36),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build());

        drive.turn(Math.toRadians(225));
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()))
                .lineTo(new Vector2d(drive.getPoseEstimate().getX() - 28.5, drive.getPoseEstimate().getY()),
                        SampleMecanumDrive.getVelocityConstraint(30, 0.5, 10.36),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build());
        motor_carusel.setPower(1);
        sleep(3000);
        motor_carusel.setPower(0);

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()))
                .lineTo(new Vector2d(drive.getPoseEstimate().getX() + 2, drive.getPoseEstimate().getY() + 40),
                        SampleMecanumDrive.getVelocityConstraint(20, 0.5, 10.36),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build());
        if (tip_autonomie == 3 || tip_autonomie == 0) {
            motor_slider.setTargetPosition(1650);
            motor_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (motor_slider.getCurrentPosition() < motor_slider.getTargetPosition() - 10)
                motor_slider.setPower(1);
            motor_slider.setPower(0);

            ElapsedTime servo_timer = new ElapsedTime();
            OTcup.setPower(0.2);
            OTcup2.setPower(-0.65);
            servo_timer.reset();
            while (servo_timer.milliseconds() < 800) {
            }
            OTcup2.setPower(0.80);

            servo_timer.reset();
            while (servo_timer.milliseconds() < 800) {
            }

            OTcup2.setPower(0.65);
            OTcup.setPower(1);

            motor_slider.setTargetPosition(0);
            while (motor_slider.getCurrentPosition() > motor_slider.getTargetPosition() + 10)
                motor_slider.setPower(1);
            motor_slider.setPower(0);

        }

        if (tip_autonomie == 2) {
            motor_slider.setTargetPosition(1275);
            motor_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (motor_slider.getCurrentPosition() < motor_slider.getTargetPosition() - 10)
                motor_slider.setPower(0.6);
            motor_slider.setPower(0);

            ElapsedTime servo_timer = new ElapsedTime();
            OTcup.setPower(-0.6);
            OTcup2.setPower(-0.95);
            servo_timer.reset();
            while (servo_timer.milliseconds() < 1500) {
            }
            OTcup2.setPower(-0.1);

            servo_timer.reset();
            while (servo_timer.milliseconds() < 900) {
            }

            OTcup2.setPower(-0.3);

            servo_timer.reset();
            while (servo_timer.milliseconds() < 500) {
            }

            OTcup.setPower(1);

            motor_slider.setTargetPosition(0);
            while (motor_slider.getCurrentPosition() > motor_slider.getTargetPosition() + 10)
                motor_slider.setPower(1);
            motor_slider.setPower(0);

        }

        if (tip_autonomie == 1) {
            motor_slider.setTargetPosition(1100);
            motor_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (motor_slider.getCurrentPosition() < motor_slider.getTargetPosition() - 10)
                motor_slider.setPower(0.6);
            motor_slider.setPower(0);

            ElapsedTime servo_timer = new ElapsedTime();
            OTcup.setPower(-0.6);
            OTcup2.setPower(-0.95);
            servo_timer.reset();
            while (servo_timer.milliseconds() < 1500) {
            }
            OTcup2.setPower(-0.1);

            servo_timer.reset();
            while (servo_timer.milliseconds() < 900) {
            }

            OTcup2.setPower(-0.3);

            servo_timer.reset();
            while (servo_timer.milliseconds() < 500) {
            }

            OTcup.setPower(1);

            motor_slider.setTargetPosition(0);
            while (motor_slider.getCurrentPosition() > motor_slider.getTargetPosition() + 10)
                motor_slider.setPower(1);
            motor_slider.setPower(0);

        }
        stick.setPower(0.2);
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()))
                .lineTo(new Vector2d(drive.getPoseEstimate().getX() - 1.75, drive.getPoseEstimate().getY() - 19.25),
                        SampleMecanumDrive.getVelocityConstraint(20, 0.5, 10.36),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build());

        OTcup2.setPower(0.65);
        sleep(2000);
        cup.setPosition(0.1);
        sleep(1000);
    }




    class functions {
        //deprecated
        private int button_sleep = 135;

        //Valoare care retine daca bratul este in reset sau nu
        private boolean resetting_brat = false;

        //Limita de jos si de sus a bratului ca inclinare
        private final int upper_limit_brat = 1400;
        private final int lower_limit_brat = -3500;

        //Valoare care retine daca bratul este destul de extins pentru inclinare si distanta de clear
        private boolean clear_brat = false;
        private int dist_min_ext = 900;

        public void run_slider(int dist)
        {
            if (dist > 2400)
                return;

            motor_slider.setTargetPosition(dist);

            while (Math.abs(Math.abs(motor_slider.getCurrentPosition()) - Math.abs(dist)) >= 20 )
                motor_slider.setPower(1);

            motor_slider.setPower(0);
            sleep(100);

        }

    }
}

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

@Autonomous(name = "OPERATION: Warehouse||Blue", group = "main")
public class BlueAuto3 extends LinearOpMode{

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
        double y = 4;
        double x = -1;
        dsensor1 = hardwareMap.get(DistanceSensor.class, "dsensor1");
        int tip_autonomie;
        waitForStart();
        double last_x;
        double last_y;
        int count = 1;
        ElapsedTime timer_autonomie = new ElapsedTime();
        tip_autonomie = pipe_line.gen_tip_autonomie();
        if (isStopRequested()) return;

        drive = new SampleMecanumDrive(hardwareMap);
        ArrayList<Trajectory> path = new ArrayList<>();
        boolean start = false;

        telemetry.addData("Autonomie: ", tip_autonomie);
        telemetry.update();

        stick.setPower(0);
        while (true) {
            path.clear();
            cup.setPosition(0.3);

            OTcup2.setPower(0.65);
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0))
                    .strafeRight(12,
                            SampleMecanumDrive.getVelocityConstraint(70, 0.5, 10.36),
                            SampleMecanumDrive.getAccelerationConstraint(70))
                    .build());

            int goto_poz_slider;
            goto_poz_slider = 1100;
            double motor_speed;
            motor_speed = 1;
            if (tip_autonomie == 1 && !start)
            {
                goto_poz_slider = 450;
                motor_speed = 0.7;
            }

            else if (tip_autonomie == 2 && !start)
            {
                goto_poz_slider = 850;
                motor_speed = 0.7;
            }

            motor_slider.setTargetPosition(goto_poz_slider);
            motor_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (motor_slider.getCurrentPosition() < motor_slider.getTargetPosition() - 10)
                motor_slider.setPower(motor_speed);
            motor_slider.setPower(0);

            ElapsedTime servo_timer = new ElapsedTime();
            if (start || tip_autonomie != 1) {
                OTcup.setPower(0.2);
                OTcup2.setPower(-0.65);
            }
            if (!start && tip_autonomie == 1)
            {
                OTcup.setPower(-0.6);
                OTcup2.setPower(-0.95);
            }

            servo_timer.reset();
            int servo_time;
            servo_time = 800;

            if (start)
                servo_time = 950;

            while (servo_timer.milliseconds() < servo_time) {
            }
            if (start || tip_autonomie != 1)
                OTcup2.setPower(0.80);
            if (!start && tip_autonomie == 1)
                OTcup2.setPower(-0.1);

            servo_timer.reset();
            while (servo_timer.milliseconds() < servo_time) {
            }

            OTcup2.setPower(0.65);
            OTcup.setPower(1);

            motor_slider.setTargetPosition(0);
            motor_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (motor_slider.getCurrentPosition() > motor_slider.getTargetPosition() + 10)
                motor_slider.setPower(1);
            motor_slider.setPower(0);

            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0))
                    .strafeLeft(13.25,
                            SampleMecanumDrive.getVelocityConstraint(70, 0.5, 10.36),
                            SampleMecanumDrive.getAccelerationConstraint(70))
                    .build());


            cup.setPosition(0.8);
            int speed_final;
            speed_final = 70;
            if (timer_autonomie.milliseconds() > 27000)
                speed_final = 37;
            path.add(drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0))
                    .lineTo(new Vector2d(30 + x, y - 4),
                            SampleMecanumDrive.getVelocityConstraint(speed_final, 0.5, 10.36),
                            SampleMecanumDrive.getAccelerationConstraint(speed_final))
                    .build());
            drive.followTrajectory(path.get(0));

            path.add(drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0))
                    .lineTo(new Vector2d(
                                    drive.getPoseEstimate().getX() + 50,
                                    drive.getPoseEstimate().getY() - 30),
                            SampleMecanumDrive.getVelocityConstraint(7, 1, 10.36),
                            SampleMecanumDrive.getAccelerationConstraint(7))
                    .build());
            drive.followTrajectoryAsync(path.get(1));

            motor_colector.setPower(1);
            OTcup2.setPower(0);
            double distance = dsensor1.getDistance(DistanceUnit.MM);
            double l_time = 0;
            ElapsedTime timer = new ElapsedTime();
            ElapsedTime blocked_timer = new ElapsedTime();
            blocked_timer.reset();
            while (distance > 50 && timer.milliseconds() < l_time + 75) {
                drive.update();
                distance = dsensor1.getDistance(DistanceUnit.MM);

                if (l_time == 0 && distance < 50) {
                    timer.reset();
                    l_time = timer.milliseconds();
                    motor_colector.setPower(0);
                }

                if (distance > 50) {
                    l_time = 0;
                    timer.reset();
                    motor_colector.setPower(1);
                }

                if (blocked_timer.milliseconds() > 3000)
                {
                    drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX() - 1, drive.getPoseEstimate().getY(), 0))
                            .strafeLeft(4,
                                    SampleMecanumDrive.getVelocityConstraint(70, 0.5, 10.36),
                                    SampleMecanumDrive.getAccelerationConstraint(70))
                            .build());
                    drive.followTrajectoryAsync(drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0))
                            .lineTo(new Vector2d(
                                            drive.getPoseEstimate().getX() + 50,
                                            drive.getPoseEstimate().getY() - 30),
                                    SampleMecanumDrive.getVelocityConstraint(7, 1, 10.36),
                                    SampleMecanumDrive.getAccelerationConstraint(7))
                            .build());
                    timer.reset();
                    blocked_timer.reset();
                }

            }
            y += 5;
            if (count <= 2)
                x -= 4.5;
            count++;
            if (timer_autonomie.milliseconds() > 21000)
            {
                drive.breakFollowing();
                break;
            }

            drive.breakFollowing();
            cup.setPosition(0.3);
            OTcup2.setPower(0);
            servo_timer.reset();
            while (servo_timer.milliseconds() < 300)
            {

            }
            motor_colector.setPower(1);

            path.add(drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0))
                    .lineTo(new Vector2d(20 + x - 6, y + 1.5),
                            SampleMecanumDrive.getVelocityConstraint(70, 0.5, 10.36),
                            SampleMecanumDrive.getAccelerationConstraint(70))
                    .build());

            drive.followTrajectory(path.get(2));
            path.add(drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0))
                    .lineTo(new Vector2d(0 + x*2, y - 3),
                            SampleMecanumDrive.getVelocityConstraint(70, 0.5, 10.36),
                            SampleMecanumDrive.getAccelerationConstraint(70))
                    .build());
            drive.followTrajectory(path.get(3));

            motor_colector.setPower(0);
            if (start == false)
                start = true;
        }
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

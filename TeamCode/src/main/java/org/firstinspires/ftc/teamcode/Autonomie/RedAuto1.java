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


//Road Runner Imports - Lucian

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "Red Side", group = "main")
public class RedAuto1 extends LinearOpMode{

    //Motoare Brat, Slidere, Carusel, Colector
    private DcMotorEx motor_brat;
    private DcMotorEx motor_slider;
    private DcMotorEx motor_carusel;
    private DcMotorEx motor_colector;

    //Variabila Dashboard
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private SampleMecanumDrive mecanum_drive;

    private CRServo holder;
    private CRServo trap;
    private CRServo stick;
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


        //Init pentru servo-ul de la holder de elemente
        holder = hardwareMap.crservo.get("holder");
        holder.setDirection(CRServo.Direction.REVERSE);

        trap = hardwareMap.crservo.get("trap");
        trap.setDirection(DcMotorSimple.Direction.REVERSE);

        stick = hardwareMap.crservo.get("stick");
        stick.setDirection(DcMotorSimple.Direction.REVERSE);

        holder.setPower(-0.3);
        trap.setPower(1);
        stick.setPower(-0.84);

        //Init pentru mecanum drive RR
        mecanum_drive = new SampleMecanumDrive(hardwareMap);

        mecanum_drive.setPoseEstimate(new Pose2d(0, 0, 0));


        int tip_autonomie;
        waitForStart();

        tip_autonomie = pipe_line.gen_tip_autonomie();
        telemetry.addData("Autonomie: ", tip_autonomie);
        telemetry.update();



        if (isStopRequested()) return;

        //Obiecte Road-Runner
        drive = new SampleMecanumDrive(hardwareMap);
        ArrayList<Trajectory> path = new ArrayList<>();
        DriveConstants driveConstants = new DriveConstants();


        path.add(drive.trajectoryBuilder(new Pose2d())
        .lineTo(new Vector2d(3, 24))
        .build());

        double last_x = path.get(0).end().getX();
        double last_y = path.get(0).end().getY();

        path.add(drive.trajectoryBuilder(path.get(0).end())
        .lineToLinearHeading(new Pose2d(last_x + 12, last_y, Math.toRadians(180)))
        .build());

        drive.followTrajectory(path.get(0));
        drive.followTrajectory(path.get(1));

        motor_slider.setTargetPositionTolerance(10);

            if (tip_autonomie == 1){
                drive.followTrajectory(drive.trajectoryBuilder(path.get(1).end())
                .forward(0.5)
                .build());

                motor_slider.setTargetPosition(870);
                motor_slider.setPower(1.35);
                while(motor_slider.isBusy());
                motor_slider.setPower(0);

                sleep(200);

                holder.setPower(-1);
                sleep(500);
                trap.setPower(0);

                sleep(1000);

                holder.setPower(-0.3);
                trap.setPower(1);

                sleep(700);

                motor_slider.setTargetPosition(0);
                motor_slider.setPower(1);
                while(motor_slider.isBusy());
                motor_slider.setPower(0);

                sleep(200);

                drive.followTrajectory(drive.trajectoryBuilder(path.get(1).end())
                        .back(0.25)
                        .build());

            }

        if (tip_autonomie == 2){
            drive.followTrajectory(drive.trajectoryBuilder(path.get(1).end())
                    .forward(0.75)
                    .build());
            motor_slider.setTargetPosition(1330);
            motor_slider.setPower(1);
            while(motor_slider.isBusy());
            motor_slider.setPower(0);

            sleep(200);

            holder.setPower(-1);
            sleep(500);
            trap.setPower(0);

            sleep(1000);

            holder.setPower(-0.3);
            trap.setPower(1);

            sleep(700);

            motor_slider.setTargetPosition(0);
            motor_slider.setPower(1);
            while(motor_slider.isBusy());
            motor_slider.setPower(0);

            sleep(200);
        }

        if (tip_autonomie == 3 || tip_autonomie == 0){
            motor_slider.setTargetPosition(2500);
            motor_slider.setPower(1);
            while(motor_slider.isBusy());
            motor_slider.setPower(0);

            sleep(200);

            holder.setPower(-1);
            sleep(500);
            trap.setPower(0);

            sleep(1000);

            holder.setPower(-0.3);
            trap.setPower(1);

            sleep(700);

            motor_slider.setTargetPosition(0);
            motor_slider.setPower(1);
            while(motor_slider.isBusy());
            motor_slider.setPower(0);

            sleep(200);
        }

        sleep(300);

        last_x = path.get(1).end().getX() - 10;
        last_y = path.get(1).end().getY();



        path.add(drive.trajectoryBuilder(path.get(1).end())
        .lineToLinearHeading(new Pose2d(last_x - 4.5, last_y + 47.5, Math.abs(405)),
                SampleMecanumDrive.getVelocityConstraint(40, 4, 10),
                SampleMecanumDrive.getAccelerationConstraint(40))
        .build());

        drive.followTrajectory(path.get(2));
        motor_carusel.setPower(-0.5);
        sleep(4500);
        motor_carusel.setPower(0);

        path.add(drive.trajectoryBuilder(path.get(2).end())
        .back(12)
        .build());

        drive.followTrajectory(path.get(3));

        path.add(drive.trajectoryBuilder(path.get(3).end())
        .lineToLinearHeading(new Pose2d(-12.2, 0, Math.toRadians(270)),
                SampleMecanumDrive.getVelocityConstraint(40, 4, 10),
                SampleMecanumDrive.getAccelerationConstraint(40))
                .build());
        drive.followTrajectory(path.get(4));

        path.add(drive.trajectoryBuilder(path.get(4).end())
        .forward(28)
        .build());
        drive.followTrajectory(path.get(5));

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

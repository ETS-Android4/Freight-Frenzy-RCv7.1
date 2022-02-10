package org.firstinspires.ftc.teamcode.Autonomie;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectObject extends OpenCvPipeline {
    Mat mat = new Mat();
    private int tip_autonomie;

    static final Rect zona_stanga = new Rect(
            new Point(00, 135),
            new Point(90, 200));

    static final Rect zona_mijloc = new Rect(
            new Point(120, 135),
            new Point(170, 200));

    static final Rect zona_dreapta = new Rect(
            new Point(200, 135),
            new Point(310, 200));


    static double procentObiectPeZona = 0.0020;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //35, 85, 94 - default
        //43, 62, 100 - brightness 100
        //35, 100, 69 - brightness 0
        //Take half of HSV online H value, and then make a range +- 10/5
        Scalar lowHSV = new Scalar(50.5, 45, 45);
        Scalar highHSV = new Scalar(80.5, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat stanga = mat.submat(zona_stanga);
        Mat mijloc = mat.submat(zona_mijloc);
        Mat dreapta = mat.submat(zona_dreapta);

        double val_stanga = Core.sumElems(stanga).val[0] / zona_stanga.area() / 255;
        double val_mijloc = Core.sumElems(mijloc).val[0] / zona_mijloc.area() / 255;
        double val_dreapta = Core.sumElems(dreapta).val[0] / zona_dreapta.area() / 255;
        stanga.release();
        mijloc.release();
        dreapta.release();

        boolean elementLeft = val_stanga > procentObiectPeZona;
        boolean elementMiddle = val_mijloc > procentObiectPeZona;
        boolean elementRight = val_dreapta > procentObiectPeZona;

        if (elementLeft) {
            tip_autonomie = 1;
        }
        else if (elementMiddle) {
            tip_autonomie = 2;
        }
        else if (elementRight){
            tip_autonomie = 3;
        }
        else
            tip_autonomie = 0;

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar nu_exista = new Scalar(255, 0, 0);
        Scalar exista = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, zona_stanga, tip_autonomie == 1 ? exista : nu_exista);
        Imgproc.rectangle(mat, zona_mijloc, tip_autonomie == 2 ? exista : nu_exista);
        Imgproc.rectangle(mat, zona_dreapta, tip_autonomie == 3 ? exista : nu_exista);

        return mat;
    }

    public int gen_tip_autonomie() {
        return tip_autonomie;
    }
}
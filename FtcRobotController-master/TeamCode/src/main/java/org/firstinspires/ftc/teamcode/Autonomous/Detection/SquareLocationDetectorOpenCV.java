package org.firstinspires.ftc.teamcode.Autonomous.Detection;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Size;
import java.util.ArrayList;

import org.opencv.core.MatOfPoint;


public class SquareLocationDetectorOpenCV extends OpenCvPipeline {
    private final Telemetry telemetry;

    private final Mat mat;
    private Mat result = null;

    private CustomElementLocation customElementLocation = CustomElementLocation.NOT_FOUND;

    public SquareLocationDetectorOpenCV(Telemetry t) {

        telemetry = t;
        mat       = new Mat();

    }

    @Override
    public Mat processFrame(Mat input) {

        //if (true){ return input; }

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HLS);


        Scalar lower = new Scalar (30, 10, 150);
        Scalar upper = new Scalar (100, 80, 255);


        Core.inRange(mat, lower, upper, mat);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

        Imgproc.erode(mat, mat, kernel);
        Imgproc.dilate(mat, mat, kernel);
        Imgproc.GaussianBlur(mat, mat, new Size(3, 3), 10);

        Imgproc.erode(mat, mat, kernel);
        Imgproc.dilate(mat, mat, kernel);
        Imgproc.GaussianBlur(mat, mat, new Size(3, 3), 10);

        Imgproc.threshold(mat, mat, 20, 255, Imgproc.THRESH_BINARY);


        ArrayList<MatOfPoint> contours = new ArrayList<>();

        Mat temp = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.findContours(mat, contours, temp, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);


        if (result != null)  result.release();
        else result = new Mat();


        if (contours.size() > 0){

            double maxVal = 0;
            int maxValIdx = -1;


            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {

                double contourArea = Imgproc.contourArea(contours.get(contourIdx));

                if (contourArea > 500 && contourArea > maxVal){

                    maxVal = contourArea;
                    maxValIdx = contourIdx;

                }
                else if ((contourArea <= 500) && (maxValIdx > -1)) {

                    contours.remove(contourIdx);

                }

            }


            Core.bitwise_and(input, input, result, mat);
            mat.release();


            if (maxValIdx >= 0) {

                Rect biggestRect = Imgproc.boundingRect(new MatOfPoint(
                        contours.get(maxValIdx).toArray()));


                Point supDir = new Point (biggestRect.x, biggestRect.y);
                Point botEsc = new Point (biggestRect.x + biggestRect.width,
                        biggestRect.y + biggestRect.height);


                Imgproc.rectangle(result, supDir, botEsc, new Scalar(0, 255, 0), 5);


                setLocation(biggestRect.x + biggestRect.width / 2);

            }
            else this.customElementLocation = CustomElementLocation.NOT_FOUND;

        }
        else {

            Core.bitwise_and(input, input, result, mat);

            mat.release();
            this.customElementLocation = CustomElementLocation.NOT_FOUND;

        }

        switch (this.getLocation()) {

            case LEFT://LEFT ~176
                Imgproc.line(result,
                        new Point(100, 190),
                        new Point(100, 230),
                        new Scalar(0, 255, 255),
                        3);
                break;

            case RIGHT://RIGHT ~515
                Imgproc.line(result,
                        new Point(430, 190),
                        new Point(430, 230),
                        new Scalar(0, 255, 255),
                        3);
                break;

            case CENTER://CENTER ~327
                Imgproc.line(result,
                        new Point(312, 190),
                        new Point(312, 230),
                        new Scalar(0, 255, 255),
                        3);
                break;

            case NOT_FOUND:
            default:
                Imgproc.line(result,
                        new Point(200, 180),
                        new Point(400, 290),
                        new Scalar(0, 255, 255),
                        3);
                Imgproc.line(result,
                        new Point(400, 180),
                        new Point(200, 290),
                        new Scalar(0, 255, 255),
                        3);
        }

        return result;

    }

    private void setLocation(int valX) {

        if (valX < 252)  {

            this.customElementLocation = CustomElementLocation.LEFT;

        }
        else if (valX > 422){

            this.customElementLocation = CustomElementLocation.RIGHT;

        }
        else {

            this.customElementLocation = CustomElementLocation.CENTER;

        }
    }

    public CustomElementLocation getLocation() {
        return this.customElementLocation;
    }

}


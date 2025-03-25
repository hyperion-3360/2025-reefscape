// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.math.util.Units;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Add your docs here. */
public class PegDetect {

  // TODO: determine me!!! 7 3/4 to 21
  private final double kHFOV = 13.25; // inches
  private final String path = "/media/sda/images/"; // path to save images

  // Threshold of violet in HSV space
  // Those will most likely need to be recalibrated with the real camera,
  // lighting and reef of the actual field
  final Scalar lower_violet = new Scalar(137, 48, 38);
  final Scalar upper_violet = new Scalar(165, 143, 134);

  final double kMinBoungingBoxWidth = 17; // pixels
  final double kMaxBoungingBoxWidth = 22; // pixels

  final double kMinBoungingBoxHeight = 22; // pixels

  // lighting and reef of workshop field
  //  final Scalar lower_violet = new Scalar(156, 72, 162);
  // final Scalar upper_violet = new Scalar(166, 121, 241);

  private CvSink m_sink;
  private double m_offset = 0;
  private boolean m_validDetection = false;
  Mat mat1 = new Mat();
  Mat mat2 = new Mat();
  List<MatOfPoint> contours = new ArrayList<>();

  public PegDetect(CvSink sink) {
    m_sink = sink;
  }

  /**
   * Get the offset of the peg from the center of the image. This is done by grabbing a frame from
   * the camera, converting it to HSV, applying a mask to only show the violet color, and then
   * finding the largest contour. The offset is then calculated by finding the center of the
   * bounding rectangle of the largest contour.
   *
   * @return a Detection object
   */
  public boolean processImage() {
    m_validDetection = false;
    try {
      if (m_sink.grabFrame(mat1) != 0) {
        File f = new File(path);
        if (f.isDirectory()) {
          var imgName = String.format("%s/%d.png", path, System.currentTimeMillis());
          System.out.println("Saving image to : " + imgName);
          Imgcodecs.imwrite(imgName, mat1);
        }

        double imageWidth = mat1.width();
        System.out.println("Frame acquired");
        Imgproc.cvtColor(mat1, mat2, Imgproc.COLOR_BGR2HSV);
        System.out.println("Color conversion to HSV done");
        Imgproc.GaussianBlur(mat2, mat1, new Size(3, 3), 0);
        System.out.println("Gaussian blur done");

        // mask to overlay
        Core.inRange(mat1, lower_violet, upper_violet, mat2);
        System.out.println("In range performed");

        contours.clear();

        Imgproc.findContours(
            mat2, contours, mat1, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        System.out.println("Find countours completed");

        double maxArea = 0;
        int maxValId = 0;

        for (var c : contours) {
          double contourArea = Imgproc.contourArea(c);
          if (maxArea < contourArea) {
            maxArea = contourArea;
            maxValId = contours.indexOf(c);
          }
        }
        System.out.println("Finding largest area completed");

        var boundingRect = Imgproc.boundingRect(contours.get(maxValId));

        System.out.println("Bounding rect: " + boundingRect.toString());

        if (boundingRect.width < kMinBoungingBoxWidth
            || boundingRect.width > kMaxBoungingBoxWidth
            || boundingRect.height < kMinBoungingBoxHeight) {
          System.out.println("Bounding box too small or too big");
          return false;
        }

        // image is vertical so left right is along the x axis
        var bbox_center = boundingRect.x + boundingRect.width / 2;
        bbox_center -= imageWidth / 2;

        System.out.println("offset from center: " + bbox_center + " max width: " + imageWidth);

        double pixToMeasureRatio = kHFOV / imageWidth;

        m_offset = Units.inchesToMeters(bbox_center * pixToMeasureRatio);

        System.out.println("offset in meters: " + m_offset);
        m_validDetection = true;
      } else {
        System.out.println("ERROR ---> Can't acquire image!!!!");
      }
    } catch (Exception e) {
      System.out.println("Caught exception : " + e);
    }

    return m_validDetection;
  }

  public double getOffset() {
    return m_offset;
  }

  public boolean isValidDetection() {
    return m_validDetection;
  }
}

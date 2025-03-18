// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.cscore.CvSink;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/** Add your docs here. */
public class PegDetect {

  // Threshold of violet in HSV space
  // Those will most likely need to be recalibrated with the real camera,
  // lighting and reef of the actual field
  final Scalar lower_violet = new Scalar(150, 60, 70);
  final Scalar upper_violet = new Scalar(175, 165, 130);

  private CvSink m_sink;
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
   * @return the offset of the peg from the center of the image
   */
  public double getOffset() {
    if (m_sink.grabFrame(mat1) != 0) {
      Imgproc.cvtColor(mat1, mat2, Imgproc.COLOR_BGR2HSV);
      Imgproc.GaussianBlur(mat2, mat1, new Size(3, 3), 0);

      // mask to overlay
      Core.inRange(mat1, lower_violet, upper_violet, mat2);

      contours.clear();

      Imgproc.findContours(mat2, contours, mat1, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

      double maxArea = 0;
      int maxValId = 0;

      for (var c : contours) {
        double contourArea = Imgproc.contourArea(c);
        if (maxArea < contourArea) {
          maxArea = contourArea;
          maxValId = contours.indexOf(c);
        }
      }

      var boundingRect = Imgproc.boundingRect(contours.get(maxValId));

      // image is vertical so left right is along the x axis
      var width_center = boundingRect.x + boundingRect.width / 2;

      return width_center;
    }

    // if we can't grab a frame return -1
    return -1;
  }
}

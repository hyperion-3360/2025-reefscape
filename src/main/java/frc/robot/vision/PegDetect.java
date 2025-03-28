// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Add your docs here. */
public class PegDetect {

  // TODO: determine me!!! 7 3/4 to 21
  private final double kHFOV = 13.25; // inches
  private String m_path = "/media/sda/images/"; // path to save images

  // Threshold of violet in HSV space
  // Those will most likely need to be recalibrated with the real camera,
  // lighting and reef of the actual field
  final Scalar klower_violet = new Scalar(154, 0, 0);
  final Scalar kupper_violet = new Scalar(171, 255, 255);
  final double kThreshold = 0.5;

  private CvSink m_sink;
  private double m_offset = 0;
  private boolean m_validDetection = false;
  Mat mat1 = new Mat();
  Mat mat2 = new Mat();
  Mat m_template = new Mat();

  public PegDetect(CvSink sink) {
    m_sink = sink;

    var numDirs = 0;

    File dir = new File(m_path);
    try {
      File[] files = dir.listFiles();

      // For-each loop for iteration
      for (File file : files)
        // Checking of file inside directory
        if (file.isDirectory()) numDirs++;
    } catch (Exception e) {
      // Printing the exception occurred
      System.out.println("Exception: " + e);
    }

    m_path += String.format("/Iteration_%d", numDirs);
    File theDir = new File(m_path);
    if (!theDir.exists()) {
      System.out.print("Creating directory: " + theDir.getName() + "...");
      System.out.println(theDir.mkdirs() ? "Success" : "Failed");
    } else {
      System.out.println("Directory already exists");
    }

    var templateFilePath = Filesystem.getDeployDirectory() + File.separator + "template_bw.png";
    m_template = Imgcodecs.imread(templateFilePath, Imgcodecs.IMREAD_GRAYSCALE);
  }

  /**
   * Process the image to detect the peg Using the template matching algorithm with a pre-defined
   * template and minimum squared difference. The offset is calculated based on the center of the
   * bounding box of the detected peg. A validation threshold is used to determine if the peg is
   * detected.
   *
   * @return true if the peg is detected, false otherwise, use the getOffset() method to get the
   *     offset
   */
  public boolean processImage() {
    m_validDetection = false;

    long grabResult = 0;

    try {
      grabResult = m_sink.grabFrame(mat1);
    } catch (Exception e) {
      System.out.println("Caught grab exception : " + e);
    }

    if (grabResult != 0) {
      try {
        File f = new File(m_path);
        if (f.isDirectory()) {
          var imgName = String.format("%s/%d.png", m_path, System.currentTimeMillis());
          System.out.println("Saving image to : " + imgName);
          Imgcodecs.imwrite(imgName, mat1);
        }
      } catch (Exception e) {
        System.out.println("Caught file exception : " + e);
      }

      try {
        double imageWidth = mat1.width();
        System.out.println("Frame acquired");
        Imgproc.cvtColor(mat1, mat2, Imgproc.COLOR_BGR2HSV);
        System.out.println("Color conversion to HSV done");
        Imgproc.medianBlur(mat2, mat1, 5);
        System.out.println("Median blur done");

        // mask to overlay
        Core.inRange(mat1, klower_violet, kupper_violet, mat2);
        System.out.println("In range performed");

        Imgproc.matchTemplate(mat1, m_template, mat2, Imgproc.TM_SQDIFF_NORMED);
        System.out.println("Template matching performed");

        var result = Core.minMaxLoc(mat2);
        System.out.println("Minmax location performed");

        if (result.minVal < kThreshold) {
          var bbox_center = result.minLoc.x + m_template.width() / 2;
          double pixToMeasureRatio = kHFOV / imageWidth;
          m_offset = Units.inchesToMeters(bbox_center * pixToMeasureRatio);

          System.out.println("offset in meters: " + m_offset);
          m_validDetection = true;
        } else {
          System.out.println("No peg detected!");
        }
      } catch (Exception e) {
        System.out.println("Caught opencv exception : " + e);
      }
    } else {
      System.out.println("Grab frame failed");
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

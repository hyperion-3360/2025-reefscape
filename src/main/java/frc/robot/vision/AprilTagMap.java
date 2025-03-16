// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;

/** Add your docs here. */
public class AprilTagMap {

  AprilTag Tag1 =
      new AprilTag(
          1,
          new Pose3d(
              Units.inchesToMeters(657.37),
              Units.inchesToMeters(25.8),
              Units.inchesToMeters(58.5),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(126.0))));
  AprilTag Tag2 =
      new AprilTag(
          2,
          new Pose3d(
              Units.inchesToMeters(657.37),
              Units.inchesToMeters(291.20),
              Units.inchesToMeters(58.5),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(234.0))));
  AprilTag Tag3 =
      new AprilTag(
          3,
          new Pose3d(
              Units.inchesToMeters(455.15),
              Units.inchesToMeters(317.15),
              Units.inchesToMeters(51.25),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(270.0))));
  AprilTag Tag4 =
      new AprilTag(
          4,
          new Pose3d(
              Units.inchesToMeters(365.2),
              Units.inchesToMeters(241.64),
              Units.inchesToMeters(73.54),
              new Rotation3d(0.0, Units.degreesToRadians(30.0), 0.0)));
  AprilTag Tag5 =
      new AprilTag(
          5,
          new Pose3d(
              Units.inchesToMeters(365.2),
              Units.inchesToMeters(75.39),
              Units.inchesToMeters(73.54),
              new Rotation3d(0.0, Units.degreesToRadians(30.0), 0.0)));
  AprilTag Tag6 =
      new AprilTag(
          6,
          new Pose3d(
              Units.inchesToMeters(530.49),
              Units.inchesToMeters(130.17),
              Units.inchesToMeters(12.13),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(300.0))));
  AprilTag Tag7 =
      new AprilTag(
          7,
          new Pose3d(
              Units.inchesToMeters(546.87),
              Units.inchesToMeters(158.5),
              Units.inchesToMeters(12.13),
              new Rotation3d(0.0, 0.0, 0.0)));
  AprilTag Tag8 =
      new AprilTag(
          8,
          new Pose3d(
              Units.inchesToMeters(530.49),
              Units.inchesToMeters(186.83),
              Units.inchesToMeters(12.13),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(60.0))));
  AprilTag Tag9 =
      new AprilTag(
          9,
          new Pose3d(
              Units.inchesToMeters(497.77),
              Units.inchesToMeters(186.83),
              Units.inchesToMeters(12.13),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(120.0))));
  AprilTag Tag10 =
      new AprilTag(
          10,
          new Pose3d(
              Units.inchesToMeters(481.39),
              Units.inchesToMeters(158.5),
              Units.inchesToMeters(12.13),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0))));
  AprilTag Tag11 =
      new AprilTag(
          11,
          new Pose3d(
              Units.inchesToMeters(497.77),
              Units.inchesToMeters(130.7),
              Units.inchesToMeters(12.13),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(240.0))));
  AprilTag Tag12 =
      new AprilTag(
          12,
          new Pose3d(
              Units.inchesToMeters(33.51),
              Units.inchesToMeters(25.8),
              Units.inchesToMeters(58.5),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(54.0))));
  AprilTag Tag13 =
      new AprilTag(
          13,
          new Pose3d(
              Units.inchesToMeters(33.51),
              Units.inchesToMeters(291.2),
              Units.inchesToMeters(58.5),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(306.0))));
  AprilTag Tag14 =
      new AprilTag(
          14,
          new Pose3d(
              Units.inchesToMeters(325.68),
              Units.inchesToMeters(241.64),
              Units.inchesToMeters(73.54),
              new Rotation3d(0.0, Units.degreesToRadians(30.0), Units.degreesToRadians(180.0))));
  AprilTag Tag15 =
      new AprilTag(
          15,
          new Pose3d(
              Units.inchesToMeters(325.68),
              Units.inchesToMeters(75.39),
              Units.inchesToMeters(73.54),
              new Rotation3d(0.0, Units.degreesToRadians(30.0), Units.degreesToRadians(180.0))));
  AprilTag Tag16 =
      new AprilTag(
          16,
          new Pose3d(
              Units.inchesToMeters(235.73),
              Units.inchesToMeters(-0.15),
              Units.inchesToMeters(51.25),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0))));
  AprilTag Tag17 =
      new AprilTag(
          17,
          new Pose3d(
              Units.inchesToMeters(160.39),
              Units.inchesToMeters(130.7),
              Units.inchesToMeters(12.13),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(240))));
  AprilTag Tag18 =
      new AprilTag(
          18,
          new Pose3d(
              Units.inchesToMeters(144.0),
              Units.inchesToMeters(158.5),
              Units.inchesToMeters(12.13),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0))));
  AprilTag Tag19 =
      new AprilTag(
          19,
          new Pose3d(
              Units.inchesToMeters(160.39),
              Units.inchesToMeters(186.83),
              Units.inchesToMeters(12.13),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(120.0))));
  AprilTag Tag20 =
      new AprilTag(
          20,
          new Pose3d(
              Units.inchesToMeters(193.1),
              Units.inchesToMeters(186.83),
              Units.inchesToMeters(12.13),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(60.0))));
  AprilTag Tag21 =
      new AprilTag(
          21,
          new Pose3d(
              Units.inchesToMeters(209.49),
              Units.inchesToMeters(158.5),
              Units.inchesToMeters(12.13),
              new Rotation3d(0.0, 0.0, 0.0)));
  AprilTag Tag22 =
      new AprilTag(
          22,
          new Pose3d(
              Units.inchesToMeters(193.1),
              Units.inchesToMeters(130.17),
              Units.inchesToMeters(12.13),
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(300.0))));

  ArrayList<AprilTag> tags = new ArrayList<>();
  AprilTagFieldLayout field;

  public AprilTagMap() {

    tags.add(Tag1);
    tags.add(Tag2);
    tags.add(Tag3);
    tags.add(Tag4);
    tags.add(Tag5);
    tags.add(Tag6);
    tags.add(Tag7);
    tags.add(Tag8);
    tags.add(Tag9);
    tags.add(Tag10);
    tags.add(Tag11);
    tags.add(Tag12);
    tags.add(Tag13);
    tags.add(Tag14);
    tags.add(Tag15);
    tags.add(Tag16);
    tags.add(Tag17);
    tags.add(Tag18);
    tags.add(Tag19);
    tags.add(Tag20);
    tags.add(Tag21);
    tags.add(Tag22);

    field =
        new AprilTagFieldLayout(
            tags, Units.inchesToMeters(690.875958), Units.inchesToMeters(317.000));
  }
}

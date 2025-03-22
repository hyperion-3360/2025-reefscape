// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;

/** Add your docs here. */
public class ElasticSetupV2 {


  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("temporary tab");

//   NetworkTableEntry help = new NetworkTableEntry(inst, 0);

  private static final StringTopic testString =  NetworkTableInstance.getDefault().getStringTopic("hello");
  private StringPublisher help = testString.publish();

  public void things() {
    testString.publish().set("things");
    help.set("hello again");

    
  }

}

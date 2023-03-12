// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private double limeX;


  /** Creates a new Limelight. */
  public Limelight() {
    limeX = 0.0;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limeX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
  }


  public double getX(){
    return limeX;
  }
}

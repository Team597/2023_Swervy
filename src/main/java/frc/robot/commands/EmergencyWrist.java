// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubystem;

public class EmergencyWrist extends CommandBase {
  private WristSubystem wrist;
  /** Creates a new EmergencyWrist. */
  public EmergencyWrist(WristSubystem w_Wrist) {
    wrist = w_Wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!wrist.limitSwitched()){
      wrist.driveWrist(-0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.driveWrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.limitSwitched();
  }
}

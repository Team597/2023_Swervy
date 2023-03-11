// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubystem;

public class DriveWrist extends CommandBase {
  private WristSubystem wrist;
  private DoubleSupplier power;
  
  
  /** Creates a new DriveWrist. */
  public DriveWrist(WristSubystem w_Wrist, DoubleSupplier w_Power) {
    // Use addRequirements() here to declare subsystem dependencies.
    wrist = w_Wrist;
    power = w_Power;

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.driveWrist(power.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

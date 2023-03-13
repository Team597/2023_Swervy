// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubystem;

public class MagicWrist extends CommandBase {
  private WristSubystem wrist;
  private int wristPosition;
  private BooleanSupplier isBox;


  /** Creates a new MagicWrist. */
  public MagicWrist(WristSubystem w_Wrist, int w_Position, BooleanSupplier w_Box) {
    // Use addRequirements() here to declare subsystem dependencies.
    wrist = w_Wrist;
    wristPosition = w_Position;
    isBox = w_Box;

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.magicset(wristPosition, isBox.getAsBoolean());
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

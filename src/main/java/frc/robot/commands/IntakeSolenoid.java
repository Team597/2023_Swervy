// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSolenoid extends CommandBase {
  private IntakeSubsystem intake;
  private boolean direction;
  /** Creates a new IntakeSolenoid. */
  public IntakeSolenoid(IntakeSubsystem w_Intake, boolean w_Dir) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = w_Intake;
    direction = w_Dir;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.toggleSolenoid(direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DriveIntake extends CommandBase {
  private IntakeSubsystem intake;
  private double power;

  /** Creates a new DriveIntake. */
  public DriveIntake(IntakeSubsystem w_Intake, double w_Power) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = w_Intake;
    power = w_Power;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.intakeMode()=="Cube" && power<=0){
      power *= 0.2;
    }
    intake.driveIntake(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.driveIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //intake.driveIntake(0);
    return false;
  }
}

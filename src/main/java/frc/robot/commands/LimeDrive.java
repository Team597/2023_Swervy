// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class LimeDrive extends CommandBase {
  private Limelight lime;
  private Swerve swerve;
  /** Creates a new LimeDrive. */
  public LimeDrive(Limelight w_Lime, Swerve w_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    lime = w_Lime;
    swerve = w_Swerve;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double strafe = -(lime.getX()/27);
    
    swerve.drive(new Translation2d(0.0, strafe).times(Constants.Swerve.maxSpeed), 0, false, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoGyroDrive extends CommandBase {
  private final Swerve swerve;
  private double x_Power, y_Power, rot_Power;

  public AutoGyroDrive(Swerve w_Swerve, double x, double y, double rot) {
    swerve = w_Swerve;
    addRequirements(swerve);

    x_Power = x;
    y_Power = y;
    rot_Power = rot;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerve.gyroDrive(
          new Translation2d(x_Power, y_Power).times(Constants.Swerve.maxSpeed), 
          rot_Power, 
          true, 
          true
      );
  }

  @Override
  public void end(boolean interrupted) {
    swerve.gyroDrive(
          new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
          rot_Power, 
          true, 
          true
      );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

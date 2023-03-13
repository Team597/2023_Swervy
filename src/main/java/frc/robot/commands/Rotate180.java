// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Rotate180 extends CommandBase {
  private Swerve s_Swerve;    
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private SlewRateLimiter translateSlew;
  private SlewRateLimiter strafeSlew;

  
  /** Creates a new Rotate180. */
  public Rotate180(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
      this.s_Swerve = s_Swerve;
      addRequirements(s_Swerve);

      this.translationSup = translationSup;
      this.strafeSup = strafeSup;
      this.rotationSup = rotationSup;
      this.robotCentricSup = robotCentricSup;

      translateSlew = new SlewRateLimiter(0.5);
      strafeSlew = new SlewRateLimiter(0.5);
  
  }

  @Override
  public void execute() {
      /* Get Values, Deadband*/
      double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
      double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
      double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
      
      translationVal = translateSlew.calculate(translationVal);
      strafeVal = strafeSlew.calculate(strafeVal);

      /* Drive */
      s_Swerve.drive(
          new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
          rotationVal * Constants.Swerve.maxAngularVelocity, 
          !robotCentricSup.getAsBoolean(), 
          true
      );
  }
}

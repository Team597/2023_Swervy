// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoPowerBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private final Swerve swerve;
  PIDController outputCalc = new PIDController(0.012, 0.0, 0.0);//0.0095, 0.0, 0.0


  public AutoPowerBalance(Swerve w_Swerve) {
    swerve = w_Swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outputCalc.setSetpoint(0);
    outputCalc.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //System.out.println("Current Pitch: " + swerve.getPitch() + " Desired Pitch: " + testnum + " PID: " + outputCalc.calculate(swerve.getPitch(), swerve.groundPitch));
    double outputPID = -outputCalc.calculate(swerve.getPitch(), swerve.groundPitch); 

    SmartDashboard.putNumber("AutoBalance", outputPID);
    if(Math.abs(swerve.getPitch())>2.0){
      /* Drive */
      swerve.gyroDrive(
          new Translation2d(outputPID, 0).times(Constants.Swerve.maxSpeed), 
          0, 
          true, 
          true
      );
    }else{
      System.out.println("balance");
      swerve.gyroDrive(
          new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
          0, 
          true, 
          true
      );
    }
    //System.out.println("Driving: " + outputPID + "Gyro: " + swerve.getPitch());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0).times(0), 
    0, 
    true, 
    true);
    System.out.println("endededed");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

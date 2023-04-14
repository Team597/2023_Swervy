// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class IncreaseElevator extends CommandBase {
  private ElevatorSubsystem elevator;
  private DoubleSupplier power;
  /** Creates a new IncreaseElevator. */
  public IncreaseElevator(ElevatorSubsystem w_Elevator, DoubleSupplier w_Power) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = w_Elevator;
    power = w_Power;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deadPower = MathUtil.applyDeadband(power.getAsDouble(), 0.05);
    deadPower = (int)deadPower * 300;
    elevator.incMagic(deadPower);
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

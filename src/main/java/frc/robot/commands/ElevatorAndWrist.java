// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorAndWrist extends ParallelCommandGroup {
  /** Creates a new ElevatorAndWrist. */
  public ElevatorAndWrist(ElevatorSubsystem ele, WristSubystem wrist, BooleanSupplier isBox, int pose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double timeDelay = 0;
    if(pose==0||pose==1||pose==4){
      timeDelay = 0.0;
    } else{
      timeDelay = 0.1;
    }

    /*if(pose!= 0){
      timeDelay = 0.1;
    } else {
      timeDelay = 0.0;
    }*/
    addCommands(new MagicElevator(ele, pose, isBox), new WaitMagicWrist(timeDelay, wrist, pose, isBox));
  }
}

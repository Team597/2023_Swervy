package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.DriveIntake;
import frc.robot.commands.ElevatorAndWrist;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristSubystem;

import java.util.List;
import java.util.function.BooleanSupplier;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class ShootAndSpinDrive extends SequentialCommandGroup {
    public ShootAndSpinDrive(Swerve s_Swerve, ElevatorSubsystem s_Elevator, WristSubystem s_Wrist, BooleanSupplier isBox, IntakeSubsystem s_Intake){
        s_Swerve.zeroGyro();
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        //PathPlannerTrajectory newExample = PathPlanner.loadPath("New New Path", new PathConstraints(3, 3));
                // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(2, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(4.5, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);


        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new ElevatorAndWrist(s_Elevator, s_Wrist, isBox, 6).withTimeout(1.75),
            new DriveIntake(s_Intake, Constants.AutoConstants.ScorePower).withTimeout(0.15),
            new ElevatorAndWrist(s_Elevator, s_Wrist, isBox, 0).withTimeout(0.5),
            new DriveIntake(s_Intake, 0).withTimeout(0.05),
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand,
            new AutoGyroDrive(s_Swerve, 0, 0, 180).withTimeout(1),
            new ParallelCommandGroup(new AutoGyroDrive(s_Swerve, 0.3, 0, 180), new ElevatorAndWrist(s_Elevator, s_Wrist, isBox, 1), new DriveIntake(s_Intake, 1)).withTimeout(1.5),
            new ParallelCommandGroup(new AutoGyroDrive(s_Swerve, -0.3, 0, 0), new ElevatorAndWrist(s_Elevator, s_Wrist, isBox, 0), new DriveIntake(s_Intake, 1)).withTimeout(2.0),
            new ElevatorAndWrist(s_Elevator, s_Wrist, isBox, 4).withTimeout(0.75),
            new DriveIntake(s_Intake, -1.0).withTimeout(0.15)
        );
    }
}
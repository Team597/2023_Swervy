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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class ShootAndCrossBalance extends SequentialCommandGroup {
    public ShootAndCrossBalance(Swerve s_Swerve, ElevatorSubsystem s_Elevator, WristSubystem s_Wrist, BooleanSupplier isBox, IntakeSubsystem s_Intake){
        s_Swerve.zeroGyro();
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond-0.5,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared-0.5)
                .setKinematics(Constants.Swerve.swerveKinematics);
        TrajectoryConfig reverseconfig =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics);

        reverseconfig.setReversed(true);


        Trajectory forward =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(4, 0)),
                new Pose2d(6, 0, new Rotation2d(0)),
                config);
        Trajectory reverse = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(-0.5, 0)),
                new Pose2d(-0.75, 0, new Rotation2d(0)),
                reverseconfig);
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);


        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                forward,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        SwerveControllerCommand reverserController =
                new SwerveControllerCommand(
                    reverse,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);

        addCommands(
            new ElevatorAndWrist(s_Elevator, s_Wrist, isBox, 6).withTimeout(2.0),
            new DriveIntake(s_Intake, Constants.AutoConstants.ScorePower).withTimeout(0.2),
            new ElevatorAndWrist(s_Elevator, s_Wrist, isBox, 0).withTimeout(0.5),
            new DriveIntake(s_Intake, 0).withTimeout(0.1),
            new InstantCommand(() -> s_Swerve.resetOdometry(forward.getInitialPose())),
            swerveControllerCommand,
            //new InstantCommand(() -> s_Swerve.resetOdometry(reverse.getInitialPose())),
            //reverserController,
            new AutoGyroDrive(s_Swerve, -0.25, 0, 0).withTimeout(0.5),
            new AutoPowerBalance(s_Swerve)
        );
    }
}
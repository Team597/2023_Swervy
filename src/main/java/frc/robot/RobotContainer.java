package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Positions;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utils.GamepadAxisButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick codriver = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final int wristAxis = XboxController.Axis.kLeftY.value;
    private final int eleAxis = XboxController.Axis.kRightY.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton limeButton = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Co-Driver Buttons */
    private final GamepadAxisButton intakePiece = new GamepadAxisButton(this::axisLeftCheck);
    private final GamepadAxisButton outtakePiece = new GamepadAxisButton(this::axisRightCheck);

    private final JoystickButton coneMode = new JoystickButton(codriver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton cubeMode = new JoystickButton(codriver, XboxController.Button.kRightBumper.value);

    private final JoystickButton lowScore = new JoystickButton(codriver, XboxController.Button.kA.value);
    private final JoystickButton midScore1 = new JoystickButton(codriver, XboxController.Button.kB.value);
    private final JoystickButton midScore2 = new JoystickButton(codriver, XboxController.Button.kX.value);
    private final JoystickButton highScore = new JoystickButton(codriver, XboxController.Button.kY.value);

    private final POVButton groundPickup = new POVButton(codriver, 180);
    private final POVButton midPickup1 = new POVButton(codriver, 270);
    private final POVButton midPickup2 = new POVButton(codriver, 90);
    private final POVButton highPickup = new POVButton(codriver, 0);

    private final JoystickButton manualWrist = new JoystickButton(codriver, XboxController.Button.kBack.value);
    private final JoystickButton manualElevator = new JoystickButton(codriver, XboxController.Button.kStart.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    private final WristSubystem s_Wrist = new WristSubystem();
    private final Limelight s_Limelight = new Limelight();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Wrist.setDefaultCommand(new MagicWrist(s_Wrist, Positions.wHome));
        s_Elevator.setDefaultCommand(new MagicElevator(s_Elevator, Positions.eHome));
        //s_Wrist.setDefaultCommand(new DriveWrist(s_Wrist, () -> -codriver.getRawAxis(wristAxis)));
       // s_Elevator.setDefaultCommand(new DriveElevator(s_Elevator, () -> -codriver.getRawAxis(eleAxis)));
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        limeButton.whileTrue(new LimeDrive(s_Limelight, s_Swerve));
        limeButton.onFalse(new TeleopSwerve(
            s_Swerve, 
            () -> driver.getRawAxis(translationAxis), 
            () -> driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            () -> robotCentric.getAsBoolean()
        ));
       

        /* CoDriver Buttons */
        manualWrist.onTrue(new DriveWrist(s_Wrist, () -> -codriver.getRawAxis(wristAxis)));
        manualElevator.onTrue(new DriveElevator(s_Elevator, () -> -codriver.getRawAxis(eleAxis)));

        intakePiece.whileTrue(new DriveIntake(s_Intake,1.0));
        outtakePiece.whileTrue(new DriveIntake(s_Intake,-1.0));

        coneMode.onTrue(new IntakeSolenoid(s_Intake, true));
        cubeMode.onTrue(new IntakeSolenoid(s_Intake, false));

        groundPickup.debounce(0.1)
            .onTrue(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 1))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 0));
        midPickup1
            .onTrue(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 2))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 0));
        midPickup2
            .onTrue(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 2))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 0));
        highPickup
            .onTrue(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 3))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 0));

        lowScore
            .onTrue(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 1))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 0));
        midScore1
            .onTrue(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 4))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 0));
        midScore2
            .onTrue(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 4))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 0));
        highScore
            .onTrue(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 5))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist, s_Intake, 0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }

    public boolean axisRightCheck(){
        return Math.abs(codriver.getRawAxis(3)) > 0.5;
    }
    public boolean axisLeftCheck(){
        return Math.abs(codriver.getRawAxis(2)) > 0.5;
    }
}

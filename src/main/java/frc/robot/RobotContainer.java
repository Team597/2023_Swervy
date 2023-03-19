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

    private final GamepadAxisButton turn0 = new GamepadAxisButton(this::driveraxisLeftCheck);
    private final GamepadAxisButton turn180 = new GamepadAxisButton(this::driveraxisRightCheck);


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
    private final POVButton pov135 = new POVButton(codriver, 135);
    private final POVButton pov225 = new POVButton(codriver, 225);
    private final POVButton midPickup1 = new POVButton(codriver, 270);
    private final POVButton midPickup2 = new POVButton(codriver, 90);
    private final POVButton pov45 = new POVButton(codriver, 45);
    private final POVButton pov315 = new POVButton(codriver, 315);
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
        s_Wrist.setDefaultCommand(new MagicWrist(s_Wrist, 0, () -> s_Intake.isBox()));
        s_Elevator.setDefaultCommand(new MagicElevator(s_Elevator, 0, () -> s_Intake.isBox()));
       // s_Wrist.setDefaultCommand(new DriveWrist(s_Wrist, () -> -codriver.getRawAxis(wristAxis)));
        //s_Elevator.setDefaultCommand(new DriveElevator(s_Elevator, () -> -codriver.getRawAxis(eleAxis)));
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
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            () -> robotCentric.getAsBoolean()
        ));
        turn0.whileTrue(new Rotate180(s_Swerve,
         () -> -driver.getRawAxis(translationAxis),
         () -> -driver.getRawAxis(strafeAxis),
         0,
         () -> robotCentric.getAsBoolean()));
        
        turn180.whileTrue(new Rotate180(s_Swerve,
         () -> -driver.getRawAxis(translationAxis),
         () -> -driver.getRawAxis(strafeAxis),
         180,
         () -> robotCentric.getAsBoolean()));
       

        /* CoDriver Buttons */
        manualWrist.onTrue(new DriveWrist(s_Wrist, () -> -codriver.getRawAxis(wristAxis)));
        manualElevator.onTrue(new DriveElevator(s_Elevator, () -> -codriver.getRawAxis(eleAxis)));

        intakePiece.whileTrue(new DriveIntake(s_Intake,1.0));
        outtakePiece.whileTrue(new DriveIntake(s_Intake,-1.0));

        coneMode.onTrue(new IntakeSolenoid(s_Intake, true));
        cubeMode.onTrue(new IntakeSolenoid(s_Intake, false));

        groundPickup.debounce(0.1)
            .whileTrue(new ElevatorAndWrist(s_Elevator, s_Wrist, () -> s_Intake.isBox(), 1))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 0));
            pov135.debounce(0.1)
            .whileTrue(new ElevatorAndWrist(s_Elevator, s_Wrist, () -> s_Intake.isBox(), 1))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 0));
            pov225.debounce(0.1)
            .whileTrue(new ElevatorAndWrist(s_Elevator, s_Wrist, () -> s_Intake.isBox(), 1))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 0));
        midPickup1.debounce(0.1)
            .whileTrue(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 2))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 0));
        midPickup2.debounce(0.1)
            .whileTrue(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 2))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 0));
            pov45.debounce(0.1)
            .whileTrue(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 2))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 0));
            pov315.debounce(0.1)
            .whileTrue(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 2))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 0));
        highPickup.debounce(0.1)
            .whileTrue(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 3))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 0));

        lowScore.debounce(0.1)
            .whileTrue(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 4))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 0));
        midScore1.debounce(0.1)
            .whileTrue(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 5))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 0));
        midScore2.debounce(0.1)
            .whileTrue(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 5))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 0));
        highScore.debounce(0.1)
            .whileTrue(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 6))
            .onFalse(new ElevatorAndWrist(s_Elevator, s_Wrist,  () -> s_Intake.isBox(), 0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve, s_Elevator, s_Wrist,  () -> s_Intake.isBox(), s_Intake);
    }

    public void resetGyro(){
        s_Swerve.zeroGyro();
    }


    public boolean axisRightCheck(){
        return Math.abs(codriver.getRawAxis(3)) > 0.5;
    }
    public boolean axisLeftCheck(){
        return Math.abs(codriver.getRawAxis(2)) > 0.5;
    }


    public boolean driveraxisRightCheck(){
        return Math.abs(driver.getRawAxis(3)) > 0.5;
    }
    public boolean driveraxisLeftCheck(){
        return Math.abs(driver.getRawAxis(2)) > 0.5;
    }
}

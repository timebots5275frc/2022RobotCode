// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autonomous.AutonomousDrive;
import frc.robot.commands.autonomous.AutonomousShootCargo;
import frc.robot.commands.multi_subsystem.IntakeCargo_AutoHopper;
import frc.robot.commands.multi_subsystem.ShootCargo_AutoHopper;
import frc.robot.commands.multi_subsystem.VisionAutoTargeting_MoveRobot;
import frc.robot.commands.single_subsystem.RunClimberExtendingArms;
import frc.robot.commands.single_subsystem.RunClimberExtendingArmsCurrent;
import frc.robot.commands.single_subsystem.RunIntake;
import frc.robot.commands.single_subsystem.RunLowerHopper;
import frc.robot.commands.single_subsystem.RunShooter;
import frc.robot.commands.single_subsystem.RunUpperHopper;
import frc.robot.commands.single_subsystem.TeleopJoystickDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here.

    // Subsystems
    public Drivetrain drivetrain = new Drivetrain();
    public Climber climber = new Climber();
    public Intake intake = new Intake();
    public Hopper hopper = new Hopper();
    public Shooter shooter = new Shooter();
    public Vision vision = new Vision();

    // Joystick
    private Joystick driveStick = new Joystick(0);
    private Joystick auxStick = new Joystick(1);

    // Autonomous Commands
    private AutonomousDrive autonomousDrive = new AutonomousDrive();
    private AutonomousShootCargo autonomousShootCargo = new AutonomousShootCargo();

    // Multi Subsystem Commands
    private IntakeCargo_AutoHopper intakeCargo_AutoHopper = new IntakeCargo_AutoHopper(intake, hopper);
    private ShootCargo_AutoHopper shootCargo_AutoHopper = new ShootCargo_AutoHopper(shooter, hopper);
    private VisionAutoTargeting_MoveRobot visionAutoTargeting_MoveRobot = new VisionAutoTargeting_MoveRobot();

    // Single Subsystem Commands
    private TeleopJoystickDrive teleopJoystickDrive = new TeleopJoystickDrive(drivetrain, driveStick, auxStick, false);
    private RunIntake runIntake = new RunIntake(intake, Constants.IntakeConstants.MOTOR_SPEED);
    private RunIntake runIntakeBackwards = new RunIntake(intake, Constants.IntakeConstants.MOTOR_REVERSE_SPEED);
    private RunLowerHopper runLowerHopper = new RunLowerHopper(hopper, Constants.HopperConstants.HOPPER_FIRE_SPEED);
    private RunLowerHopper runLowerHopperBackwards = new RunLowerHopper(hopper,
            Constants.HopperConstants.HOPPER_BACK_SPEED);
    private RunUpperHopper runUpperHopper = new RunUpperHopper(hopper, Constants.HopperConstants.HOPPER_FIRE_SPEED);
    private RunUpperHopper runUpperHopperBackwards = new RunUpperHopper(hopper,
            Constants.HopperConstants.HOPPER_BACK_SPEED);

    private RunShooter runShooterFast = new RunShooter(shooter, Constants.ShooterConstants.UPPER_PORT_SHOOTER_FIRE_RPM);
    private RunClimberExtendingArms runClimberExtendingArms = new RunClimberExtendingArms(climber);
    private RunClimberExtendingArmsCurrent runClimberExtendingArmsCurrent = new RunClimberExtendingArmsCurrent(climber);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        drivetrain.setDefaultCommand(teleopJoystickDrive);
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(driveStick, 2).whenHeld(runIntake, true);
        new JoystickButton(driveStick, 4).whenHeld(runLowerHopper, true);
        new JoystickButton(driveStick, 6).whenHeld(runUpperHopper, true);
        new JoystickButton(driveStick, 1).whenHeld(runShooterFast, true);
        new JoystickButton(driveStick, 5).whenHeld(runClimberExtendingArms, true);
        new JoystickButton(driveStick, 11).whenHeld(runClimberExtendingArmsCurrent, true);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}

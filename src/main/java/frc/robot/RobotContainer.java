// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
import frc.robot.constants.Constants.AutoConstants;
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

    // Multi Subsystem Commands
    private IntakeCargo_AutoHopper intakeCargo_AutoHopper = new IntakeCargo_AutoHopper(intake, hopper);
    private ShootCargo_AutoHopper shootCargo_AutoHopper = new ShootCargo_AutoHopper(shooter, hopper,
            Constants.ShooterConstants.UPPER_PORT_SHOOTER_FIRE_RPM_1);
    private VisionAutoTargeting_MoveRobot visionAutoTargeting_MoveRobot = new VisionAutoTargeting_MoveRobot();

    // Single Subsystem Commands
    private TeleopJoystickDrive teleopJoystickDrive = new TeleopJoystickDrive(drivetrain, driveStick, auxStick, true);
    private RunIntake runIntake = new RunIntake(intake, Constants.IntakeConstants.MOTOR_SPEED);
    private RunIntake runIntakeBackwards = new RunIntake(intake, Constants.IntakeConstants.MOTOR_REVERSE_SPEED);
    private RunLowerHopper runLowerHopper = new RunLowerHopper(hopper, Constants.HopperConstants.HOPPER_FIRE_SPEED);
    private RunLowerHopper runLowerHopperBackwards = new RunLowerHopper(hopper,
            Constants.HopperConstants.HOPPER_BACK_SPEED);
    private RunUpperHopper runUpperHopper = new RunUpperHopper(hopper, Constants.HopperConstants.HOPPER_FIRE_SPEED);
    private RunUpperHopper runUpperHopperBackwards = new RunUpperHopper(hopper,
            Constants.HopperConstants.HOPPER_BACK_SPEED);
    private RunShooter runShooterFast = new RunShooter(shooter,
            Constants.ShooterConstants.UPPER_PORT_SHOOTER_FIRE_RPM_2);
    private RunClimberExtendingArms runExtendingArmsHIGH = new RunClimberExtendingArms(climber, 320);
    private RunClimberExtendingArms runExtendingArmsMID = new RunClimberExtendingArms(climber, 10);
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
        new JoystickButton(driveStick, 4).whenHeld(runIntakeBackwards, true);
        new JoystickButton(driveStick, 4).whenHeld(runLowerHopperBackwards, true);
        new JoystickButton(driveStick, 4).whenHeld(runUpperHopperBackwards, true);

        new JoystickButton(driveStick, 2).whenHeld(intakeCargo_AutoHopper, true);
        new JoystickButton(driveStick, 1).whenHeld(runShooterFast, true);

        new JoystickButton(driveStick, 3).whenHeld(runLowerHopper, true);
        new JoystickButton(driveStick, 3).whenHeld(runUpperHopper, true);

        // new JoystickButton(driveStick, 4).whenHeld(runLowerHopper, true);
        // new JoystickButton(driveStick, 6).whenHeld(runUpperHopper, true);
        new JoystickButton(driveStick, 12).whenHeld(runExtendingArmsHIGH, true);
        new JoystickButton(driveStick, 11).whenHeld(runExtendingArmsMID, true);
        new JoystickButton(driveStick, 10).whenHeld(runClimberExtendingArmsCurrent, true);

        new JoystickButton(driveStick, 7).whenPressed(() -> drivetrain.resetPIgeonIMU());
        new JoystickButton(driveStick, 7).whenPressed(() -> drivetrain.resetOdometry());
    }

    // Autonomous Commands
    private AutonomousDrive autonomousDrive = new AutonomousDrive();
    private AutonomousShootCargo autonomousShootCargo = new AutonomousShootCargo();

    public PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    public PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    public ProfiledPIDController thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController,
            0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        System.out.println("getAutonomousCommand");
        // Create config for trajectory
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_Speed_MetersPerSecond,
                AutoConstants.MAX_Acceleration_MetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(drivetrain.kinematics);

        // An example trajectory to follow. All units in meters.
        List<Pose2d> list = List.of(new Pose2d(0, 0, new Rotation2d()), new Pose2d(1, 0, new Rotation2d()));

        Trajectory exampleTrajectory = Drivetrain.generateTrajectory(config, list);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
                drivetrain::getPose,
                drivetrain.kinematics,
                // Position controllers
                xController, yController, thetaController, drivetrain::setModuleStates, drivetrain);

        drivetrain.resetOdometryWithPose2d(exampleTrajectory.getInitialPose());

        return swerveControllerCommand; // .andThen(() -> drivetrain.drive(0, 0, 0, false));

    }
}

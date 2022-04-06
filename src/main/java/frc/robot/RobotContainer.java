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
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autonomous.AutonomousDrive;
import frc.robot.commands.autonomous.AutonomousHopper;
import frc.robot.commands.autonomous.AutonomousHopperLower;
import frc.robot.commands.autonomous.AutonomousHopperUpper;
import frc.robot.commands.autonomous.AutonomousIntake;
import frc.robot.commands.autonomous.AutonomousShootCargo;
import frc.robot.commands.multi_subsystem.IntakeCargo_AutoHopper;
import frc.robot.commands.multi_subsystem.ShootCargo_AutoHopper;
import frc.robot.commands.multi_subsystem.VisionAutoTargeting_MoveRobot;
import frc.robot.commands.single_subsystem.RunClimberExtendingArms;
import frc.robot.commands.single_subsystem.RunClimberExtendingArmsCurrent;
import frc.robot.commands.single_subsystem.RunDrive;
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

    // Autonomous Commands
    // private AutonomousDrive autonomousDrive = new AutonomousDrive(drivetrain);
    public AutonomousShootCargo autonomousShootCargo = new AutonomousShootCargo(shooter, 3800);
    public AutonomousHopper autonomousHopperCargo = new AutonomousHopper(hopper,
            Constants.HopperConstants.HOPPER_FIRE_SPEED);
    public AutonomousHopperUpper autonomousHopperUpperCargo = new AutonomousHopperUpper(hopper,
            Constants.HopperConstants.HOPPER_FIRE_SPEED);
    public AutonomousHopperLower autonomousHopperLowerCargo = new AutonomousHopperLower(hopper,
            Constants.HopperConstants.HOPPER_FIRE_SPEED);
    public AutonomousIntake autonomousIntakeCargo = new AutonomousIntake(intake,
            Constants.HopperConstants.HOPPER_FIRE_SPEED);

    // Multi Subsystem Commands
    private IntakeCargo_AutoHopper intakeCargo_AutoHopper = new IntakeCargo_AutoHopper(intake, hopper);
    // private ShootCargo_AutoHopper shootCargo_AutoHopper = new
    // ShootCargo_AutoHopper(shooter, hopper,
    // Constants.ShooterConstants.UPPER_PORT_SHOOTER_FIRE_RPM_1);
    private VisionAutoTargeting_MoveRobot visionAutoTargeting_MoveRobot = new VisionAutoTargeting_MoveRobot();

    // Single Subsystem Commands
    private TeleopJoystickDrive teleopJoystickDrive = new TeleopJoystickDrive(drivetrain, driveStick, auxStick,
            true /* Feild/Robot */);
    private RunIntake runIntake = new RunIntake(intake, Constants.IntakeConstants.MOTOR_SPEED);
    private RunIntake runIntakeBackwards = new RunIntake(intake, Constants.IntakeConstants.MOTOR_REVERSE_SPEED);
    private RunLowerHopper runLowerHopper = new RunLowerHopper(hopper, Constants.HopperConstants.HOPPER_FIRE_SPEED);
    private RunLowerHopper runLowerHopperBackwards = new RunLowerHopper(hopper,
            Constants.HopperConstants.HOPPER_BACK_SPEED);
    public RunUpperHopper runUpperHopper = new RunUpperHopper(hopper, Constants.HopperConstants.HOPPER_FIRE_SPEED);
    private RunUpperHopper runUpperHopperBackwards = new RunUpperHopper(hopper,
            Constants.HopperConstants.HOPPER_BACK_SPEED);
    private RunClimberExtendingArms runExtendingArmsHIGH = new RunClimberExtendingArms(climber, 320);
    private RunClimberExtendingArms runExtendingArmsMID = new RunClimberExtendingArms(climber, 10);
    public RunClimberExtendingArmsCurrent runClimberExtendingArmsCurrent = new RunClimberExtendingArmsCurrent(climber);

    private RunShooter runShooterLower = new RunShooter(shooter, 1500, auxStick);
    public RunShooter runShooterUpperOnLine1 = new RunShooter(shooter, 4000, auxStick); // 3500rpm for line
    private RunShooter runShooterUpperBall2 = new RunShooter(shooter, 5000, auxStick);
    private RunShooter runShooterUpperClimber3 = new RunShooter(shooter, 5000, auxStick);

    private RunDrive runDrive = new RunDrive(drivetrain);
    public int autoSelect = 0;

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
        new JoystickButton(driveStick, 1).whenHeld(runShooterUpperOnLine1, true);
        new JoystickButton(driveStick, 2).whenHeld(intakeCargo_AutoHopper, true);

        new JoystickButton(driveStick, 5).whenHeld(runShooterLower);

        new JoystickButton(driveStick, 3).whenHeld(runLowerHopper, true);
        new JoystickButton(driveStick, 3).whenHeld(runUpperHopper, true);

        new JoystickButton(driveStick, 4).whenHeld(runIntakeBackwards, true);
        new JoystickButton(driveStick, 4).whenHeld(runLowerHopperBackwards, true);
        new JoystickButton(driveStick, 4).whenHeld(runUpperHopperBackwards, true);

        // new JoystickButton(driveStick, 4).whenHeld(runLowerHopper, true);
        // new JoystickButton(driveStick, 6).whenHeld(runUpperHopper, true);
        new JoystickButton(driveStick, 10).whenHeld(runExtendingArmsHIGH, true);
        new JoystickButton(driveStick, 9).whenHeld(runExtendingArmsMID, true);

        // new JoystickButton(driveStick, 6).whenHeld(runDrive);
        new JoystickButton(driveStick, 12).whenPressed(() -> teleopJoystickDrive.setFieldRelative(false));
        new JoystickButton(driveStick, 12).whenReleased(() -> teleopJoystickDrive.setFieldRelative(true));

        new JoystickButton(driveStick, 8).whenHeld(runClimberExtendingArmsCurrent, true);

        new JoystickButton(driveStick, 7).whenPressed(() -> drivetrain.resetPIgeonIMU());
        new JoystickButton(driveStick, 7).whenPressed(() -> drivetrain.resetOdometry());
        // new JoystickButton(auxStick, 11).whenPressed(() -> );
        // new JoystickButton(auxStick, 12).whenPressed(() -> setAutoSelect(2));

    }

    public void checkButtons() {
        if (auxStick.getRawButtonPressed(11)) {
            setAutoSelect(1);
        }
        if (auxStick.getRawButtonPressed(12)) {
            setAutoSelect(2);
        }

    }

    public PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    public PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    public ProfiledPIDController thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController,
            0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public void setAutoSelect(int num) {
        autoSelect = num;
        System.out.println("Set Auto to: " + num);
    }

    public Command getAutoCommand() {
        if (autoSelect == 1) {
            return makeSingleShotAutoCommand();
        } else if (autoSelect == 2) {
            return makeTwoShotAutoCommand();
        } else {
            return null;
        }
    }

    private SequentialCommandGroup makeTwoShotAutoCommand() {
        System.out.println("makeTwoShotAutoCommand");
        // Create config for trajectory
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_Speed_MetersPerSecond,
                AutoConstants.MAX_Acceleration_MetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(drivetrain.kinematics);

        TrajectoryConfig config2 = new TrajectoryConfig(AutoConstants.MAX_Speed_MetersPerSecond2,
                AutoConstants.MAX_Acceleration_MetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(drivetrain.kinematics);

        config.setReversed(false);
        // An example trajectory to follow. All units in meters.
        List<Pose2d> list = List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(1.5, 0, Rotation2d.fromDegrees(0)));

        List<Pose2d> list2 = List.of(new Pose2d(1.5, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(1.8, 0, Rotation2d.fromDegrees(90)));

        List<Pose2d> list3 = List.of(new Pose2d(1.8, 0, Rotation2d.fromDegrees(90)),
                new Pose2d(1.4, 0, Rotation2d.fromDegrees(180)));

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(list, config);

        Trajectory exampleTrajectory2 = TrajectoryGenerator.generateTrajectory(list2, config);

        Trajectory exampleTrajectory3 = TrajectoryGenerator.generateTrajectory(list3, config2);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
                drivetrain::getPose,
                drivetrain.kinematics,
                // Position controllers
                xController, yController, thetaController, drivetrain::setModuleStates, drivetrain);

        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(exampleTrajectory2,
                drivetrain::getPose,
                drivetrain.kinematics,
                // Position controllers
                xController, yController, thetaController, drivetrain::setModuleStates, drivetrain);

        SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(exampleTrajectory3,
                drivetrain::getPose,
                drivetrain.kinematics,
                // Position controllers
                xController, yController, thetaController, drivetrain::setModuleStates, drivetrain);

        drivetrain.resetPIgeonIMU();

        drivetrain.resetOdometryWithPose2d(exampleTrajectory.getInitialPose());

        return new SequentialCommandGroup(swerveControllerCommand, swerveControllerCommand2, swerveControllerCommand3)
                .andThen(() -> drivetrain.drive(0, 0, 0, false));

    }

    private Command makeSingleShotAutoCommand() {
        System.out.println("makeSingleShotAutoCommand");
        // Create config for trajectory
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_Speed_MetersPerSecond,
                AutoConstants.MAX_Acceleration_MetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(drivetrain.kinematics);

        config.setReversed(true);

        // An example trajectory to follow. All units in meters.
        List<Pose2d> list = List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(-1.5, 0, Rotation2d.fromDegrees(0)));

        Trajectory exampleTrajectory = Drivetrain.generateTrajectory(config, list);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
                drivetrain::getPose,
                drivetrain.kinematics,
                // Position controllers
                xController, yController, thetaController, drivetrain::setModuleStates, drivetrain);

        drivetrain.resetPIgeonIMU();

        drivetrain.resetOdometryWithPose2d(exampleTrajectory.getInitialPose());

        return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false));
    }

}

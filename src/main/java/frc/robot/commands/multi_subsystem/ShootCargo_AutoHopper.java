// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multi_subsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

/*
 * To do list Be able to use the left and right motors to shoot the ball Be able to move the hopper
 * based on beam break sensors to automatically move other balls into position
 */

public class ShootCargo_AutoHopper extends CommandBase {
    /** Creates a new ShootCargoAndAutoHopper. */
    private Shooter shooter;
    private Hopper hopper;
    private double shooterRPM;

    public ShootCargo_AutoHopper(Shooter _shooter, Hopper _hopper, double _shooterRPM) {
        // Use addRequirements() here to declare subsystem dependencies.
        shooter = _shooter;
        hopper = _hopper;
        shooterRPM = _shooterRPM;
        addRequirements(_shooter);
        addRequirements(_hopper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setShooterMotorSpeed(shooterRPM);
        hopper.runLowerHopper(Constants.HopperConstants.HOPPER_FIRE_SPEED);
        hopper.runUpperHopper(Constants.HopperConstants.HOPPER_FIRE_SPEED);

        // if (hopper.upperHopperStatus == false) {
        // hopper.runUpperHopper(Constants.HopperConstants.HOPPER_FIRE_SPEED);
        // }
        // if (hopper.lowerHopperStatus == false) {
        // hopper.runLowerHopper(Constants.HopperConstants.HOPPER_FIRE_SPEED);
        // }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setShooterMotorSpeed(Constants.ShooterConstants.SHOOTER_DEFAULT_RPM);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

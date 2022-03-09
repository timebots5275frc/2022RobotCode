// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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

    public ShootCargo_AutoHopper(Shooter _shooter, Hopper _hopper) {
        // Use addRequirements() here to declare subsystem dependencies.
        shooter = _shooter;
        hopper = _hopper;
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
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

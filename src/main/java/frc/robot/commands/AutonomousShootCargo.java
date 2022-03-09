// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command Procedure 1. Start Shooter Motor 2. Wait until Shooter Motor is at
 * full speed. 3. Wait
 * for vision to see if we are on target (if we have it working)
 */
public class AutonomousShootCargo extends CommandBase {
    /** Creates a new AutonomousShootCargo. */
    public AutonomousShootCargo() {
        // Use addRequirements() here to declare subsystem dependencies.
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

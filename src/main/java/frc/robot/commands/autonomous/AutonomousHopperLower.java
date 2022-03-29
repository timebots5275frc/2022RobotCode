// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hopper.Hopper;

public class AutonomousHopperLower extends CommandBase {
    private Hopper hopper;
    private double direction;

    /** Creates a new AutonomousHopperLower. */
    public AutonomousHopperLower(Hopper _hopper, double forwardOrBackward) {
        hopper = _hopper;
        direction = forwardOrBackward;
        // addRequirements(_hopper);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        hopper.runLowerHopper(direction);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hopper.runLowerHopper(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

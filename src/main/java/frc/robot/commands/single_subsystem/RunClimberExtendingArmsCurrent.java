// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.single_subsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class RunClimberExtendingArmsCurrent extends CommandBase {

    private Climber climber;

    /** Creates a new RunClimberExtendingArmsCurrent. */
    public RunClimberExtendingArmsCurrent(Climber _climber) {

        this.climber = _climber;

        addRequirements(climber);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.setLeftExtendingArmCurrent(-12);
        climber.setRightExtendingArmCurrent(-12);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.resetLeftExtendingArm();

        climber.setLeftExtendingArmCurrent(0);
        climber.setRightExtendingArmCurrent(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

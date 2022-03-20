// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.single_subsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class RunClimberExtendingArms extends CommandBase {

    private Climber climber;
    private double setPosition;

    /** Creates a new RunClimberExtendingArms. */
    public RunClimberExtendingArms(Climber _climber, double _setPosition) {

        this.climber = _climber;
        this.setPosition = _setPosition;

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
        climber.setLeftExtendingArmPosition(setPosition);
        climber.setRightExtendingArmPosition(setPosition);
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

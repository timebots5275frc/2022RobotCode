// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.single_subsystem;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.RotatingClimb;

public class RunRotatingArms extends CommandBase {
    /** Creates a new RunRotatingArmsForward. */
    private RotatingClimb rotatingArms;
    private double position;

    public RunRotatingArms(RotatingClimb _rotatingArms, double _position) {
        // Use addRequirements() here to declare subsystem dependencies.
        rotatingArms = _rotatingArms;
        position = _position;
        addRequirements(rotatingArms);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rotatingArms.getCurrentArmPosition();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Command running ");
        rotatingArms.moveRotArms(position);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        rotatingArms.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

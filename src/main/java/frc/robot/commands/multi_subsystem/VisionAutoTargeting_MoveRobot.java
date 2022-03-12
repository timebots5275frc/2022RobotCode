// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multi_subsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

/* To-do list
Be able to rotate the robot with the Drivetrain to point toward the reflective tape.
Be abe to get the x and y coordianates and the size of reflective tape
Possibly get the number for the shooter speed
*/

public class VisionAutoTargeting_MoveRobot extends CommandBase {
    /** Creates a new VisionAutoTargetingMoveRobot. */
    public VisionAutoTargeting_MoveRobot() {
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

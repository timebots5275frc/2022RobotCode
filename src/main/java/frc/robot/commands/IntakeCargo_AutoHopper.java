// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;


/** 
 * Command Procedure 
 * Button For Intaking Cargo
 * Start IntakeCargo_AutoHopper Command
 *   1. Start Intake Motor
 *   2. If both hopper positions are empty → move both hopper motors
 *   3. If ball in hopper position 2 → move new ball to hooper position 1
 *   4. If a ball is in both positions → the intake motor will be disabled.
 */
public class IntakeCargo_AutoHopper extends CommandBase {
  /** Creates a new IntakeCargoAutoHopper. */
  private Intake intake;
  private Hopper hopper;

  public IntakeCargo_AutoHopper(Intake _intake, Hopper _hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = _intake;
    hopper = _hopper;
    addRequirements(_intake);
    addRequirements(_hopper);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

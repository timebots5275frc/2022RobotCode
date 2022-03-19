// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multi_subsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;

/**
 * Command Procedure Button For Intaking Cargo Start IntakeCargo_AutoHopper
 * Command 1. Start Intake
 * Motor 2. If both hopper positions are empty → move both hopper motors 3. If
 * ball in hopper
 * position 2 → move new ball to hooper position 1 4. If a ball is in both
 * positions → the intake
 * motor will be disabled.
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
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (hopper.upperHopperStatus == true) { // Ball is NOT blocking upperHopper sensor
            hopper.runLowerHopper(Constants.HopperConstants.HOPPER_INTAKE_SPEED);
            hopper.runUpperHopper(Constants.HopperConstants.HOPPER_INTAKE_SPEED);
            intake.runIntakeMotor(Constants.IntakeConstants.MOTOR_SPEED);

        } else { // Ball is blocking upperHopper sensor
            hopper.runUpperHopper(0);

            if (hopper.lowerHopperStatus == true) { // Ball is NOT blocking lowerHopper sensor
                hopper.runLowerHopper(Constants.HopperConstants.HOPPER_INTAKE_SPEED);
                intake.runIntakeMotor(Constants.IntakeConstants.MOTOR_SPEED);

            } else { // Ball is blocking lowerHopper sensor
                hopper.runLowerHopper(0);
                intake.runIntakeMotor(0);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hopper.runLowerHopper(0);
        hopper.runUpperHopper(0);
        intake.runIntakeMotor(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

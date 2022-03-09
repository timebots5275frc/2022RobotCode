// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    /** Creates a new Intake. */
    VictorSPX intakeVictor = new VictorSPX(Constants.IntakeConstants.MOTOR_CAN_ID);

    public Intake() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setIntakeMotor(double percentage) {
        intakeVictor.set(ControlMode.PercentOutput, percentage);
    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Hopper extends SubsystemBase {
    VictorSPX lowerHopperMotor = new VictorSPX(Constants.HopperConstants.LOWER_HOPPER_MOTOR_ID);
    VictorSPX upperHopperMotor = new VictorSPX(Constants.HopperConstants.UPPER_HOPPER_MOTOR_ID);
    DigitalInput upperHopperSensor = new DigitalInput(9);
    DigitalInput lowerHopperSensor = new DigitalInput(8);
    boolean upperHopperStatus;
    boolean lowerHopperStatus;

    /** Creates a new Hopper. */
    public Hopper() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        upperHopperStatus = upperHopperSensor.get();
        lowerHopperStatus = lowerHopperSensor.get();
        if (upperHopperStatus == true) {
            System.out.println("Upper Working");
        }
        if (lowerHopperStatus == true) {
            System.out.println("Lower Working");
        }
    }

    public void runLowerHopper(double percentage) {
        lowerHopperMotor.set(ControlMode.PercentOutput, percentage);
    }

    public void runUpperHopper(double percentage) {
        upperHopperMotor.set(ControlMode.PercentOutput, percentage);
    }
}

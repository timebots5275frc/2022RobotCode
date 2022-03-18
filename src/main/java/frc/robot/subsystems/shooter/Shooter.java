// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.constants.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public CANSparkMax shooterRightSparkMax;
    public CANSparkMax shooterLeftSparkMax;

    private SparkMaxPIDController shooterRight_pidController;
    private SparkMaxPIDController shooterLeft_pidController;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    /** Creates a new Shooter. */
    public Shooter() {
        shooterRightSparkMax = new CANSparkMax(Constants.ShooterConstants.SHOOTER_RIGHT_MOTOR_ID,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        shooterLeftSparkMax = new CANSparkMax(Constants.ShooterConstants.SHOOTER_LEFT_MOTOR_ID,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        shooterRightSparkMax.setClosedLoopRampRate(1);
        shooterLeftSparkMax.setClosedLoopRampRate(1);

        /**
         * In order to use PID functionality for a controller, a CANPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        shooterRight_pidController = shooterRightSparkMax.getPIDController();
        shooterLeft_pidController = shooterLeftSparkMax.getPIDController();

        // Encoder object created to display position values
        // m_encoder = shooterMotor.getEncoder();

        // PID coefficients
        kP = 0.0004;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.00017; // .000015
        kMaxOutput = 1;
        kMinOutput = -1;

        // set PID coefficients shooterRight
        shooterRight_pidController.setP(kP);
        shooterRight_pidController.setI(kI);
        shooterRight_pidController.setD(kD);
        shooterRight_pidController.setIZone(kIz);
        shooterRight_pidController.setFF(kFF);
        shooterRight_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // set PID coefficients shooterRight
        shooterLeft_pidController.setP(kP);
        shooterLeft_pidController.setI(kI);
        shooterLeft_pidController.setD(kD);
        shooterLeft_pidController.setIZone(kIz);
        shooterLeft_pidController.setFF(kFF);
        shooterLeft_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        // SmartDashboard.putNumber("P Gain", kP);
        // SmartDashboard.putNumber("I Gain", kI);
        // SmartDashboard.putNumber("D Gain", kD);
        // SmartDashboard.putNumber("I Zone", kIz);
        // SmartDashboard.putNumber("Feed Forward", kFF);
        // SmartDashboard.putNumber("Max Output", kMaxOutput);
        // SmartDashboard.putNumber("Min Output", kMinOutput);

    }

    public void setShooterMotorSpeed(double input) {
        if (input < Constants.ShooterConstants.SHOOTER_MAX_RPM) {
            shooterRight_pidController.setReference(-input, ControlType.kVelocity);
            shooterLeft_pidController.setReference(-input, ControlType.kVelocity);
        } else {
            System.out.println("Shooter Motor Warning - Cannot Set Motor RPM Over Limit Of "
                    + Constants.ShooterConstants.SHOOTER_MAX_RPM);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

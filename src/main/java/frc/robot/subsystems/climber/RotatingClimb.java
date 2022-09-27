// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class RotatingClimb extends SubsystemBase {
    /** Creates a new RotatingClimb. */
    private CANSparkMax rotatingSparkMax;
    private SparkMaxPIDController rotatingSparkMaxPIDController;

    public double ra_kP, ra_kI, ra_kD, ra_kIz, ra_kFF, ra_kMaxOutput, ra_kMinOutput, ra_maxRPM, ra_smartMAXVelocity,
            ra_smartMAXAcc, ra_allowedErr;

    public RotatingClimb() {
        rotatingSparkMax = new CANSparkMax(Constants.ClimberConstants.ROTATING_CLIMBER_MOTOR_ID,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        rotatingSparkMax.getEncoder().setPosition(0);
        rotatingSparkMaxPIDController = rotatingSparkMax.getPIDController();

        // PID coefficients
        ra_kP = 0.00016;
        ra_kI = 0;
        ra_kD = 0;
        ra_kIz = 0;
        ra_kFF = 0.0;
        ra_kMaxOutput = .75;
        ra_kMinOutput = -.75;
        ra_smartMAXVelocity = 7500;
        ra_smartMAXAcc = 7500;
        ra_allowedErr = 2;

        // set PID coefficients shooterRight
        rotatingSparkMaxPIDController.setP(0.008, 1);// Current control loop
        rotatingSparkMaxPIDController.setI(0, 1);// Current control loop
        rotatingSparkMaxPIDController.setD(0, 1);// Current control loop

        rotatingSparkMaxPIDController.setP(ra_kP, 0);
        rotatingSparkMaxPIDController.setI(ra_kI, 0);
        rotatingSparkMaxPIDController.setD(ra_kD, 0);
        rotatingSparkMaxPIDController.setIZone(ra_kIz, 0);
        rotatingSparkMaxPIDController.setFF(ra_kFF, 0);
        rotatingSparkMaxPIDController.setOutputRange(ra_kMinOutput, ra_kMaxOutput, 0);
        rotatingSparkMaxPIDController.setSmartMotionMaxVelocity(ra_smartMAXVelocity, 0);
        rotatingSparkMaxPIDController.setSmartMotionMaxAccel(ra_smartMAXAcc, 0);
        rotatingSparkMaxPIDController.setSmartMotionAllowedClosedLoopError(ra_allowedErr, 0);
        rotatingSparkMaxPIDController.setSmartMotionMinOutputVelocity(10, 0);

        // robit
    }

    public void setRotatingArmPosition(double setPosition) {

        double limitedSetPosition = setPosition;

        if (setPosition > Constants.ClimberConstants.EXTENDING_CLIMBER_MAX_LIMIT) {
            limitedSetPosition = Constants.ClimberConstants.EXTENDING_CLIMBER_MAX_LIMIT;
        } else if (setPosition < Constants.ClimberConstants.EXTENDING_CLIMBER_MIN_LIMIT) {
            limitedSetPosition = Constants.ClimberConstants.EXTENDING_CLIMBER_MIN_LIMIT;
        }

        rotatingSparkMaxPIDController.setReference(limitedSetPosition, ControlType.kSmartMotion, 0);
    }

    public void setLrotatingArmCurrent(double setCurrent) {
        rotatingSparkMaxPIDController.setReference(setCurrent, ControlType.kCurrent, 1);
    }

    public void resetLrotatingExtendingArm() {
        rotatingSparkMax.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

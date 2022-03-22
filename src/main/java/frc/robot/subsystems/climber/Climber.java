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

public class Climber extends SubsystemBase {

    private CANSparkMax leftExtendingSparkMax;
    private CANSparkMax rightExtendingSparkMax;

    private SparkMaxPIDController leftExtending_pidController;
    private SparkMaxPIDController rightExtending_pidController;

    public double ex_kP, ex_kI, ex_kD, ex_kIz, ex_kFF, ex_kMaxOutput, ex_kMinOutput, ex_maxRPM, ex_smartMAXVelocity,
            ex_smartMAXAcc, ex_allowedErr;

    /** Creates a new Climber. */
    public Climber() {

        leftExtendingSparkMax = new CANSparkMax(Constants.ClimberConstants.LEFT_EXTENDING_CLIMBER_MOTOR_ID,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        rightExtendingSparkMax = new CANSparkMax(Constants.ClimberConstants.RIGHT_EXTENDING_CLIMBER_MOTOR_ID,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        rightExtendingSparkMax.getEncoder().setPosition(0);
        leftExtendingSparkMax.getEncoder().setPosition(0);
        leftExtendingSparkMax.setInverted(true);

        // leftExtendingSparkMax.setSoftLimit(SoftLimitDirection.kForward, 36 * 360);
        // leftExtendingSparkMax.setSoftLimit(SoftLimitDirection.kReverse, 0);

        leftExtending_pidController = leftExtendingSparkMax.getPIDController();
        rightExtending_pidController = rightExtendingSparkMax.getPIDController();

        leftExtending_pidController.setReference(0, ControlType.kCurrent);
        rightExtending_pidController.setReference(0, ControlType.kCurrent);

        // PID coefficients
        ex_kP = 0.00016;
        ex_kI = 0;
        ex_kD = 0;
        ex_kIz = 0;
        ex_kFF = 0.0;
        ex_kMaxOutput = .75;
        ex_kMinOutput = -.75;
        ex_smartMAXVelocity = 5000;
        ex_smartMAXAcc = 5000;
        ex_allowedErr = 2;

        // set PID coefficients shooterRight
        leftExtending_pidController.setP(0.008, 1);// Current control loop
        leftExtending_pidController.setI(0, 1);// Current control loop
        leftExtending_pidController.setD(0, 1);// Current control loop

        leftExtending_pidController.setP(ex_kP, 0);
        leftExtending_pidController.setI(ex_kI, 0);
        leftExtending_pidController.setD(ex_kD, 0);
        leftExtending_pidController.setIZone(ex_kIz, 0);
        leftExtending_pidController.setFF(ex_kFF, 0);
        leftExtending_pidController.setOutputRange(ex_kMinOutput, ex_kMaxOutput, 0);
        leftExtending_pidController.setSmartMotionMaxVelocity(ex_smartMAXVelocity, 0);
        leftExtending_pidController.setSmartMotionMaxAccel(ex_smartMAXAcc, 0);
        leftExtending_pidController.setSmartMotionAllowedClosedLoopError(ex_allowedErr, 0);
        leftExtending_pidController.setSmartMotionMinOutputVelocity(10, 0);

        // set PID coefficients shooterRight
        rightExtending_pidController.setP(0.008, 1); // Current control loop
        rightExtending_pidController.setI(0, 1);// Current control loop
        rightExtending_pidController.setD(0, 1);// Current control loop

        rightExtending_pidController.setP(ex_kP, 0);
        rightExtending_pidController.setI(ex_kI, 0);
        rightExtending_pidController.setD(ex_kD, 0);
        rightExtending_pidController.setIZone(ex_kIz, 0);
        rightExtending_pidController.setFF(ex_kFF, 0);
        rightExtending_pidController.setOutputRange(ex_kMinOutput, ex_kMaxOutput, 0);
        rightExtending_pidController.setSmartMotionMaxVelocity(ex_smartMAXVelocity, 0);
        rightExtending_pidController.setSmartMotionMaxAccel(ex_smartMAXAcc, 0);
        rightExtending_pidController.setSmartMotionAllowedClosedLoopError(ex_allowedErr, 0);
        rightExtending_pidController.setSmartMotionMinOutputVelocity(10, 0);

    }

    // public void setRightExtendingArmPosition(double setPosition) {

    // double currentEncoderPosition =
    // rightExtendingSparkMax.getEncoder().getPosition();

    // if (Math.abs(setPosition - currentEncoderPosition) < 20) {
    // System.out.println("The RightExtendingArm has reached the Position.");
    // rightExtending_pidController.setReference(0,
    // ControlType.kVelocity);

    // } else if (currentEncoderPosition < setPosition) { // Motor will go Forward
    // rightExtending_pidController.setReference(Constants.ClimberConstants.CLIMBER_EXTENDING_ARM_MOTOR_RPM,
    // ControlType.kVelocity);
    // System.out.println("Motor will go Forward " + (setPosition -
    // currentEncoderPosition));

    // } else if (currentEncoderPosition > setPosition) { // Motor will go Backward
    // rightExtending_pidController.setReference(-Constants.ClimberConstants.CLIMBER_EXTENDING_ARM_MOTOR_RPM,
    // ControlType.kVelocity);
    // System.out.println("Motor will go Backward " + (setPosition -
    // currentEncoderPosition));

    // }
    // }

    public void setRightExtendingArmPosition(double setPosition) {

        double limitedSetPosition = setPosition;

        if (setPosition > Constants.ClimberConstants.EXTENDING_CLIMBER_MAX_LIMIT) {
            limitedSetPosition = Constants.ClimberConstants.EXTENDING_CLIMBER_MAX_LIMIT;
        } else if (setPosition < Constants.ClimberConstants.EXTENDING_CLIMBER_MIN_LIMIT) {
            limitedSetPosition = Constants.ClimberConstants.EXTENDING_CLIMBER_MIN_LIMIT;
        }

        rightExtending_pidController.setReference(limitedSetPosition, ControlType.kSmartMotion, 0);
    }

    public void setRightExtendingArmCurrent(double setCurrent) {
        rightExtending_pidController.setReference(setCurrent, ControlType.kCurrent, 1);
    }

    public void setLeftExtendingArmPosition(double setPosition) {

        double limitedSetPosition = setPosition;

        if (setPosition > Constants.ClimberConstants.EXTENDING_CLIMBER_MAX_LIMIT) {
            limitedSetPosition = Constants.ClimberConstants.EXTENDING_CLIMBER_MAX_LIMIT;
        } else if (setPosition < Constants.ClimberConstants.EXTENDING_CLIMBER_MIN_LIMIT) {
            limitedSetPosition = Constants.ClimberConstants.EXTENDING_CLIMBER_MIN_LIMIT;
        }

        leftExtending_pidController.setReference(limitedSetPosition, ControlType.kSmartMotion, 0);
    }

    public void setLeftExtendingArmCurrent(double setCurrent) {
        leftExtending_pidController.setReference(setCurrent, ControlType.kCurrent, 1);
    }

    public void resetLeftExtendingArm() {
        leftExtendingSparkMax.getEncoder().setPosition(0);
    }

    public void resetRightExtendingArm() {
        rightExtendingSparkMax.getEncoder().setPosition(0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("rightExtendingSparkMax getPosition()",
                rightExtendingSparkMax.getEncoder().getPosition());

        SmartDashboard.putNumber("rightExtendingSparkMax getVelocity()",
                rightExtendingSparkMax.getEncoder().getVelocity());

        SmartDashboard.putNumber("leftExtendingSparkMax getPosition()",
                leftExtendingSparkMax.getEncoder().getPosition());

        SmartDashboard.putNumber("leftExtendingSparkMax getVelocity()",
                leftExtendingSparkMax.getEncoder().getVelocity());

        SmartDashboard.putNumber("leftExtendingSparkMax getMotorTemperature",
                leftExtendingSparkMax.getMotorTemperature());
        SmartDashboard.putNumber("leftExtendingSparkMax getOutputCurrent",
                leftExtendingSparkMax.getOutputCurrent());
        // This method will be called once per scheduler run
    }
}

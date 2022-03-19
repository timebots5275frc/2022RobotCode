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

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

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

        // PID coefficients
        kP = 0.004;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.0;
        kMaxOutput = 1;
        kMinOutput = -1;

        // set PID coefficients shooterRight
        leftExtending_pidController.setP(kP);
        leftExtending_pidController.setI(kI);
        leftExtending_pidController.setD(kD);
        leftExtending_pidController.setIZone(kIz);
        leftExtending_pidController.setFF(kFF);
        leftExtending_pidController.setOutputRange(kMinOutput, kMaxOutput);
        leftExtending_pidController.setSmartMotionMaxVelocity(2500, 0);

        // set PID coefficients shooterRight
        rightExtending_pidController.setP(kP);
        rightExtending_pidController.setI(kI);
        rightExtending_pidController.setD(kD);
        rightExtending_pidController.setIZone(kIz);
        rightExtending_pidController.setFF(kFF);
        rightExtending_pidController.setOutputRange(kMinOutput, kMaxOutput);
        rightExtending_pidController.setSmartMotionMaxVelocity(2500, 0);

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

        rightExtending_pidController.setReference(limitedSetPosition, ControlType.kPosition, 0, 0);
    }

    public void setRightExtendingArmCurrent(double setCurrent) {
        rightExtending_pidController.setReference(setCurrent, ControlType.kCurrent);
    }

    public void setLeftExtendingArmPosition(double setPosition) {

        double limitedSetPosition = setPosition;

        if (setPosition > Constants.ClimberConstants.EXTENDING_CLIMBER_MAX_LIMIT) {
            limitedSetPosition = Constants.ClimberConstants.EXTENDING_CLIMBER_MAX_LIMIT;
        } else if (setPosition < Constants.ClimberConstants.EXTENDING_CLIMBER_MIN_LIMIT) {
            limitedSetPosition = Constants.ClimberConstants.EXTENDING_CLIMBER_MIN_LIMIT;
        }

        leftExtending_pidController.setReference(limitedSetPosition, ControlType.kPosition);
    }

    public void setLeftExtendingArmCurrent(double setCurrent) {
        leftExtending_pidController.setReference(setCurrent, ControlType.kCurrent);
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
        // This method will be called once per scheduler run
    }
}

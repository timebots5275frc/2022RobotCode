// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
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
    private CANCoder angleEncoder;
    private double currentAngle;

    public double ra_kP, ra_kI, ra_kD, ra_kIz, ra_kFF, ra_kMaxOutput, ra_kMinOutput, ra_maxRPM, ra_smartMAXVelocity,
            ra_smartMAXAcc, ra_allowedErr;

    public RotatingClimb() {
        rotatingSparkMax = new CANSparkMax(Constants.ClimberConstants.ROTATING_CLIMBER_MOTOR_ID,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        angleEncoder = new CANCoder(Constants.ClimberConstants.ROTATING_CLIMBER_ENCODER_ID);
        rotatingSparkMax.getEncoder().setPosition(0);
        rotatingSparkMaxPIDController = rotatingSparkMax.getPIDController();
        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        // PID coefficients
        ra_kP = 0.0004;
        ra_kI = 0;
        ra_kD = 0;
        ra_kIz = 0;
        ra_kFF = 0.00017; // .000015
        ra_kMaxOutput = 1;
        ra_kMinOutput = -1;

        // set PID coefficients shooterRight
        rotatingSparkMaxPIDController.setP(ra_kP);
        rotatingSparkMaxPIDController.setI(ra_kI);
        rotatingSparkMaxPIDController.setD(ra_kD);
        rotatingSparkMaxPIDController.setIZone(ra_kIz);
        rotatingSparkMaxPIDController.setFF(ra_kFF);
        rotatingSparkMaxPIDController.setOutputRange(ra_kMinOutput, ra_kMaxOutput);

        // robit
    }

    public void moveRotArmsForward(double speed) {
        if (currentAngle < Constants.ClimberConstants.ROTATING_CLIMBER_FORWARD_LIMIT) {
            rotatingSparkMaxPIDController.setReference(-speed, ControlType.kVelocity);
        }

    }

    public void moveRotArmsBackward(double speed) {
        if (currentAngle > Constants.ClimberConstants.ROTATING_CLIMBER_BACKWARD_LIMIT) {
            rotatingSparkMaxPIDController.setReference(speed, ControlType.kVelocity);
        }
    }

    public void resetRotatingArm(double speed) {
        if (currentAngle < -0.5) {
            rotatingSparkMaxPIDController.setReference(speed, ControlType.kVelocity);
        }

        if (currentAngle > 0.5) {
            rotatingSparkMaxPIDController.setReference(-speed, ControlType.kVelocity);
        }
    }

    @Override
    public void periodic() {
        currentAngle = angleEncoder.getAbsolutePosition();
        // This method will be called once per scheduler run
    }
}

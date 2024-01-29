// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex
import com.revrobotics.RelativeEncoder;;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsytem extends SubsystemBase {

  CANSparkMax pivot = new CANSparkMax(Constants.PIVOT_MOTOR_ID, MotorType.kBrushless);
  CANSparkFlex bottomShooter; = new CANSparkFlex(Constants.BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
  CANSparkFlex topShooter = new CANSparkFlex(Constants.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax Feed = new CANSparkMax(Constants.FEED_MOTOR_ID, MotorType.kBrushless);
  SparkPIDController bottomShooterPID, topShooterPID, f;
  public RelativeEncoder shooterTopEncoder, shooterBottomEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, velocityRPM;
  
  
  //int     shooterAnglePID;

  /** Creates a new ShooterSubsytem. */
  public ShooterSubsytem(int sparkChannel, int talonFXDeviceId, int shooterAnglePID) {
    topShooterPID = topShooter.getPIDController();
    bottomShooterPID = bottomShooter.getPIDController();

    shooterTopEncoder = topShooter.getEncoder();
    shooterBottomEncoder = bottomShooter.getEncoder();
    //this.shooterAnglePID = shooterAnglePID;
    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000180;
    kMaxOutput = 1;
    kMinOutput = -1;
    velocityRPM = 3000; //Made this number up

    topShooterPID.setP(kP);
    topShooterPID.setI(kI);
    topShooterPID.setD(kD);
    topShooterPID.setIZone(kIz);
    topShooterPID.setFF(kFF);
    topShooterPID.setOutputRange(kMinOutput, kMaxOutput);

    bottomShooterPID.setP(kP);
    bottomShooterPID.setI(kI);
    bottomShooterPID.setD(kD);
    bottomShooterPID.setIZone(kIz);
    bottomShooterPID.setFF(kFF);
    bottomShooterPID.setOutputRange(kMinOutput, kMaxOutput);
    
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Velocity", velocityRPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void feedmotorpower(double power) {
    
  }

  public void set(double speed) {
    
  }

  

}

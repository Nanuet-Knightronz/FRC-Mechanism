// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax; 

import edu.wpi.first.math.util.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.lang.Thread.State;

import com.revrobotics.RelativeEncoder;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  private SparkMax elevatorMotor;
  private final ProfiledPIDController elevatorPID;

  private final RelativeEncoder elevatorEncoder;

  private final TrapezoidProfile.Constraints elevatorConstraints = 
    new TrapezoidProfile.Constraints(Constants.PIDConstants.PIDConstraints.kMaxVelocity, Constants.PIDConstants.PIDConstraints.kMaxAcceleration);


  public ElevatorSubsystem() {

    elevatorMotor = new SparkMax(4, MotorType.kBrushless);
    elevatorMotor.configure(null, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorEncoder = elevatorMotor.getEncoder();

    elevatorPID = new ProfiledPIDController(Constants.PIDConstants.Kp, Constants.PIDConstants.Ki, 
      Constants.PIDConstants.Kd, elevatorConstraints);

    elevatorPID.setTolerance(1.0);
  }

  public Command autoElevator(double position) {
    //no railgun
    double targetPosition = MathUtil.clamp(position, 0, 75);
    return run(()-> {
      double elevatorPosition = elevatorEncoder.getPosition();
      double ElevatorPID = elevatorPID.calculate(elevatorPosition, targetPosition);
      double motorOutput = MathUtil.clamp(ElevatorPID, -.3, .3);

        elevatorMotor.set(motorOutput);
    });
  }

  
  //use these for manual control
  public Command raiseElevatorCMD() {
    return runEnd(()-> raiseElevator(), ()-> idleElevator());
  }

  public Command lowerElevatorCMD() {
    return runEnd(()-> lowerElevator(), ()-> idleElevator());
  }

  //manual op methods
  public void raiseElevator() {
    elevatorMotor.set(.3);
  }

  public void lowerElevator() {
    elevatorMotor.set(-.4);
  }

  public void idleElevator() {
    double elevatorPosition = elevatorEncoder.getPosition();
    elevatorPID.reset(elevatorPosition);
    double idleElevatorPID = elevatorPID.calculate(elevatorPosition, elevatorPosition);
    double gravityFF = .05;

    double motorOutput = MathUtil.clamp(idleElevatorPID, -.3, .3);
      elevatorMotor.set(motorOutput += gravityFF);
  }

  public void resetEncoder() {
    elevatorEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

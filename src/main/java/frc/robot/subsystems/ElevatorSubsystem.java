// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LaserCanConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

import com.revrobotics.spark.SparkMax; 

import edu.wpi.first.math.util.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.lang.Thread.State;

import com.revrobotics.RelativeEncoder;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  private SparkMax elevatorMotor;
  private final ProfiledPIDController elevatorPID;

  private final RelativeEncoder elevatorEncoder;

  private final TrapezoidProfile.Constraints elevatorConstraints = 
    new TrapezoidProfile.Constraints(Constants.PIDConstants.PIDConstraints.kMaxVelocity, 
                                     Constants.PIDConstants.PIDConstraints.kMaxAcceleration);

  private final ElevatorFeedforward elevatorFeedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kElevatorkS,
          ElevatorConstants.kElevatorkG,
          ElevatorConstants.kElevatorkV,
          ElevatorConstants.kElevatorkA);
  
  private final LaserCan elevatorLaserCan = new LaserCan(0);
  private final RegionOfInterest elevatorLaserCanROI = new RegionOfInterest(0, 0, 16, 16);
  private final TimingBudget ElevatorLaserCanTimingBudget = TimingBudget.TIMING_BUDGET_20MS;
  private final Alert ElevatorLaserCanFail = new Alert("LaserCAN failed", AlertType.kWarning);
  
  private double targetPosition = 0.0;


  public ElevatorSubsystem() {

    elevatorMotor = new SparkMax(4, MotorType.kBrushless);
    elevatorMotor.configure(null, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorEncoder = elevatorMotor.getEncoder();

    elevatorPID = new ProfiledPIDController(Constants.PIDConstants.Kp, Constants.PIDConstants.Ki, 
      Constants.PIDConstants.Kd, elevatorConstraints);

    elevatorPID.setTolerance(1.0);


    try {
      elevatorLaserCan.setRangingMode(RangingMode.LONG);
    } catch (Exception e) {
      ElevatorLaserCanFail.set(true);
    }
  }

  public void elevatorLaserHeightCalibration() {
    Measurement seedMeasurement = elevatorLaserCan.getMeasurement();
    while (seedMeasurement == null) {
      seedMeasurement = elevatorLaserCan.getMeasurement();
    }
    elevatorEncoder.setPosition(convertDistanceToRotations(Millimeters.of(elevatorLaserCan.
        getMeasurement().distance_mm - 
        LaserCanConstants.kLaserCANOffset.in(Millimeters))).
        in(Rotations));
  }

  public Command autoElevator(double positionMeters) {
    targetPosition = MathUtil.clamp(positionMeters, 0, 0.75); // Save and clamp to 75 cm in meters
    
    return run(() -> {
      double pidOutput = elevatorPID.calculate(getElevatorPosition(), targetPosition);
      double ffOutput = elevatorFeedforward.calculateWithVelocities(getElevatorVelocity(),
                                                                    elevatorPID.getSetpoint().velocity);
      double motorVoltage = MathUtil.clamp(pidOutput + ffOutput, -7, 7);
    
      elevatorMotor.setVoltage(motorVoltage);
    });
  }

  public Command holdPositionCommand() {
    return run(() -> {
      double pidOutput = elevatorPID.calculate(getElevatorPosition(), targetPosition);
      double ffOutput = elevatorFeedforward.calculateWithVelocities(getElevatorVelocity(),
                                                                    elevatorPID.getSetpoint().velocity);
      double motorVoltage = MathUtil.clamp(pidOutput + ffOutput, -7, 7);
    
      elevatorMotor.setVoltage(motorVoltage);
    });
  }

  public double getElevatorPosition() {
    //find elevator position from encoder value and elevator drum
    return (elevatorEncoder.getPosition() / ElevatorConstants.kElevatorGearing) * (2 * Math.PI * Constants.ElevatorConstants.kDrumRadius);
  }

  public double getElevatorVelocity() {
    //get elevator velocity in m/s
    return (elevatorEncoder.getVelocity() / 60 / ElevatorConstants.kElevatorGearing) * (2 * Math.PI * Constants.ElevatorConstants.kDrumRadius);
  }

  public static Angle convertDistanceToRotations(Distance distance)
    {
      // m/(2*pi*r)*g = e
      return Rotations.of((distance.in(Meters) /
                          (ElevatorConstants.kDrumRadius * 2.0 * Math.PI)) *
                          ElevatorConstants.kElevatorGearing);
    }

  public static Distance convertRotationsToDistance(Angle rotations)
  {
      return Meters.of((rotations.in(Rotations) / ElevatorConstants.kElevatorGearing) *
                       (ElevatorConstants.kDrumRadius * 2 * Math.PI));
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
    elevatorLaserHeightCalibration();
  }
}

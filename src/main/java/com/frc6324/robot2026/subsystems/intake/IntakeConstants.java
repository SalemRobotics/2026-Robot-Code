package com.frc6324.robot2026.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

@UninstantiableClass
public final class IntakeConstants {
  private IntakeConstants() {
    throw new IllegalAccessError();
  }

  // Intake CAN constants
  public static final CANBus INTAKE_CAN_BUS = Constants.CANIVORE;
  public static final int INTAKE_DEPLOY_MOTOR_ID = 20;
  public static final int INTAKE_ROLLER_LEADER_ID = 21;
  public static final int INTAKE_ROLLER_FOLLOWER_ID = 22;

  public static final Distance INTAKE_WIDTH = Inches.of(27);
  public static final Distance INTAKE_EXTENSION = Inches.of(12);

  // Deployment setpoints
  public static final Angle INTAKE_DEPLOYED_POSITION = Rotations.of(1);
  public static final Angle INTAKE_DEPLOY_TOLERANCE = Degrees.of(2);
  public static final Angle INTAKE_STOWED_POSITION = Rotations.of(0);

  // TalonFX configurations
  public static final TalonFXConfiguration INTAKE_DEPLOY_MOTOR_CONFIG = new TalonFXConfiguration();
  public static final TalonFXConfiguration INTAKE_ROLLER_MOTOR_CONFIG = new TalonFXConfiguration();
  public static final MotorAlignmentValue INTAKE_ROLLER_MOTOR_ALIGNMENT = MotorAlignmentValue.Opposed;

  // Simulation constants for the extension/retraction motor
  public static final double INTAKE_DEPLOY_MOI = 0.2;
  public static final double INTAKE_DEPLOY_REDUCTION = 9;
  public static final DCMotor INTAKE_DEPLOY_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final MotorType INTAKE_DEPLOY_MOTOR_TYPE = MotorType.KrakenX60;

  // Simulation constants for the intake's rollers
  public static final double INTAKE_ROLLER_MOI = 0.1;
  public static final double INTAKE_ROLLER_REDUCTION = 2;
  public static final DCMotor INTAKE_ROLLER_GEARBOX = DCMotor.getKrakenX44Foc(2);
  public static final MotorType INTAKE_ROLLER_MOTOR_TYPE = MotorType.KrakenX44;
}

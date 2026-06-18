package com.frc6324.robot2026.subsystems.rollers;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import lombok.experimental.UtilityClass;

@UtilityClass
public final class RollerConstants {
  public static final CANBus ROLLER_CAN_BUS = Constants.RIO_BUS;
  public static final int ROLLER_LEADER_ID = 21;
  public static final int ROLLER_FOLLOWER_ID = 22;

  public static final TalonFXConfiguration ROLLER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(Amps.of(50))
                  .withSupplyCurrentLimitEnable(true))
          .withSlot0(new Slot0Configs().withKP(5).withKD(0.001).withKS(1).withKV(1));
  public static final MotorAlignmentValue ROLLER_ALIGNMENT = MotorAlignmentValue.Opposed;
  public static final AngularVelocity ROLLER_SPIN_VELOCITY = RPM.of(6000);
  public static final AngularVelocity ROLLER_OUTTAKE_VELOCITY = RPM.of(-6000);

  public static final double ROLLER_MOI = 0.1;
  public static final double ROLLER_REDUCTION = 2;
  public static final MotorType ROLLER_MOTOR_TYPE = MotorType.KrakenX44;
  public static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX44Foc(2);
}

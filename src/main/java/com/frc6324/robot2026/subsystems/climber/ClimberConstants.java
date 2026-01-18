package com.frc6324.robot2026.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

@UninstantiableClass
public final class ClimberConstants {
    private ClimberConstants() {
        throw new IllegalAccessError();
    }
    
    public static final CANBus CLIMBER_CAN_BUS = Constants.CANIVORE;
    public static final int CLIMBER_MOTOR_ID = 50;

    public static final double CLIMBER_DEPLOY_OUTPUT_PERCENTAGE = 1;
    public static final double CLIMBER_STOW_OUTPUT_PERCENTAGE = -1;

    public static final CurrentLimitsConfigs CLIMBER_CURRENT_LIMITS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(100))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(60))
        .withSupplyCurrentLimitEnable(true);
    
    public static final MotorType CLIMBER_MOTOR_TYPE = MotorType.KrakenX60;
    
    public static final double CLIMBER_SIM_KV = 1;
    public static final double CLIMBER_SIM_KA = 0.8;

    public static final LinearSystem<N2, N1, N2> CLIMBER_LINEAR_SYSTEM = LinearSystemId.createDCMotorSystem(CLIMBER_SIM_KV, CLIMBER_SIM_KA);
    public static final DCMotor CLIMBER_GEARBOX = DCMotor.getKrakenX60Foc(1);
}

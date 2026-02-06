package com.frc6324.lib.io.beambreak;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import lombok.Getter;

/**
 * An extension of {@link BeamBreakIODIO} that adds a glitch filter that can reduce noise directly in the RoboRIO's FPGA.
 * 
 * <p> Because of limitations in the RoboRIO's FPGA, only 3 glitch filters can be created for a whole robot. If this limit is exceeded, an exception is thrown.
 */
public class BeamBreakIOFilteredDIO extends BeamBreakIODIO {
    /**
     * The FPGA glitch filter being used.
     */
    @Getter
    private final DigitalGlitchFilter filter;

    /**
     * Creates a new beam break sensor using a real digital I/O (DIO) device.
     * 
     * <p>The caller of this function is responsible for setting the amount of time the sensor must report a value for before the FPGA propogates the new value.
     * @param dioDevice The Digital I/O device to read.
     * @param filter The filter to add this sensor to.
     * @param inverted Whether the sensor is inverted.
     */
    public BeamBreakIOFilteredDIO(DigitalInput dioDevice, DigitalGlitchFilter filter, boolean inverted) {
        super(dioDevice, inverted);

        this.filter = filter;
        filter.add(sensor);
    }

    /**
     * Creates a new beam break sensor using a real digital I/O (DIO) device.
     * @param dioDevice The Digital I/O device to read.
     * @param steadyTime The minimum time a sensor needs to maintain a state before the FPGA propogates it to the robot code.
     * @param inverted Whether the sensor is inverted.
     */
    public BeamBreakIOFilteredDIO(DigitalInput dioDevice, Time steadyTime, boolean inverted) {
        super(dioDevice, inverted);

        filter = new DigitalGlitchFilter();
        filter.add(sensor);

        final double periodSeconds = steadyTime.in(Seconds) * 1e9;
        filter.setPeriodNanoSeconds((long) periodSeconds);
    }

    /**
     * Creates a new beam break sensor using a real digital I/O (DIO) device.
     * @param dioDevice The Digital I/O device to read.
     * @param steadyTime The minimum time a sensor needs to maintain a state before the FPGA propogates it to the robot code.
     */
    public BeamBreakIOFilteredDIO(DigitalInput dioDevice, Time steadyTime) {
        this(dioDevice, steadyTime, false);
    }
    
    /**
     * Creates a new beam break sensor using a real digital I/O (DIO) device.
     * @param dioPort The DIO port the device is on. Note that if the device is disconnected, the behavior of this I/O layer is not guaranteed.
     * @param steadyTime The minimum time a sensor needs to maintain a state before the FPGA propogates it to the robot code.
     * @param inverted Whether the sensor on {@code dioPort} is inverted.
     */
    public BeamBreakIOFilteredDIO(int dioPort, Time steadyTime, boolean inverted) {
        this(new DigitalInput(dioPort), steadyTime, inverted);
    }

    /**
     * Creates a new beam break sensor using a real digital I/O (DIO) device.
     * @param dioPort The DIO port the device is on. Note that if the device is disconnected, the behavior of this I/O layer is not guaranteed.
     * @param steadyTime The minimum time a sensor needs to maintain a state before the FPGA propogates it to the robot code.
     */
    public BeamBreakIOFilteredDIO(int dioPort, Time steadyTime) {
        this(dioPort, steadyTime, false);
    }
}

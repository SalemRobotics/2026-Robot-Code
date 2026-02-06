package com.frc6324.lib.io.beambreak;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;

public final class BeamBreakIOFilteredDIO extends BeamBreakIODIO {
    private final DigitalGlitchFilter filter;

    public BeamBreakIOFilteredDIO(DigitalInput dioDevice, Time steadyTime, boolean inverted) {
        super(dioDevice, inverted);

        filter = new DigitalGlitchFilter();
        filter.add(sensor);

        final double periodSeconds = steadyTime.in(Seconds) / 1e9;
        filter.setPeriodNanoSeconds((long) periodSeconds);

    }

    public BeamBreakIOFilteredDIO(DigitalInput dioDevice, Time steadyTime) {
        this(dioDevice, steadyTime, false);
    }

    public BeamBreakIOFilteredDIO(int dioPort, Time steadyTime, boolean inverted) {
        this(new DigitalInput(dioPort), steadyTime, inverted);
    }

    public BeamBreakIOFilteredDIO(int dioPort, Time steadyTime) {
        this(dioPort, steadyTime, false);
    }
}

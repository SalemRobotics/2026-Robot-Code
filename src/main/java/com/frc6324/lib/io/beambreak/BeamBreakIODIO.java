package com.frc6324.lib.io.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIODIO implements BeamBreakIO {
    protected final DigitalInput sensor;
    private final boolean inverted;

    public BeamBreakIODIO(DigitalInput dioDevice, boolean inverted) {
        this.sensor = dioDevice;
        this.inverted = inverted;
    }

    public BeamBreakIODIO(DigitalInput dioDevice) {
        this(dioDevice, false);
    }

    public BeamBreakIODIO(int dioPort, boolean inverted) {
        this(new DigitalInput(dioPort), inverted);
    }

    public BeamBreakIODIO(int dioPort) {
        this(new DigitalInput(dioPort), false);
    }

    @Override
    public void updateInputs(BeamBreakInputs inputs) {
        inputs.connected = true;
        inputs.broken = Boolean.logicalXor(inverted, sensor.get());
    }
}

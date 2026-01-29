package com.frc6324.robot2026.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public final class IntakeMechanism {
    private static IntakeMechanism mechanism = null;

    // TODO: used LoggedMechanism2d instead(?)
    private final Mechanism2d mech = new Mechanism2d(100, 60);
    private final MechanismRoot2d intakeRoot = mech.getRoot("Intake Root", 0, 0);
    private final MechanismLigament2d intake = intakeRoot.append(new MechanismLigament2d("Intake", 0, 0));
    
    public static IntakeMechanism getInstance() {
        if (mechanism == null) {
            mechanism = new IntakeMechanism();
        }

        return mechanism;
    }

    public void setExtension(double extensionDistanceMeters) {
        // TODO: is this right????
        intake.setLength(extensionDistanceMeters);
    }

    public void log() {
        // TODO: How?
    }
}

package frc.robot.subsystems;

import frc.robot.InCANDevice;

public class Lights extends InCANDevice {
    public Lights(int dn) {
        super(dn);
        this.apiID = 25 << 4;
    }

    public void runProgram(int progn) {
        sendData(new byte[] { new Integer(progn).byteValue() });
    }

    @Override
    public void periodic() {
    }
}

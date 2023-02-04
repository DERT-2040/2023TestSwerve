package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    public LEDSubsystem() {
        m_led = new AddressableLED(1);
        m_ledBuffer = new AddressableLEDBuffer(6);
        m_led.setLength(m_ledBuffer.getLength());

        setColor(true);
        m_led.start();

    }

    public void setColor(boolean yellow) {
        if(yellow) {
            for(int i=0; i<m_ledBuffer.getLength(); i++) {
                //sets the LEDs to yellow
                m_ledBuffer.setRGB(i, 70, 25, 0);
            }
        } else {
            for(int i=0; i<m_ledBuffer.getLength(); i++) {
                //sets the LEDs to purple
                m_ledBuffer.setRGB(i, 80, 0, 40);
            }
        }
        m_led.setData(m_ledBuffer);
    }
}
    

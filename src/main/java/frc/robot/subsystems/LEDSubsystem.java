package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    int counter;

    public LEDSubsystem() {
        m_led = new AddressableLED(1);
        m_ledBuffer = new AddressableLEDBuffer(6);
        m_led.setLength(m_ledBuffer.getLength());

        setColor(true);
        m_led.start();
        counter = 0;

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
        SmartDashboard.putBoolean("LED Color", yellow);
    }


    public void idlePattern() {
        counter++;
        for(int i=0; i<m_ledBuffer.getLength(); i++) {
            //sets the LEDs to yellow
            
            m_ledBuffer.setRGB(i, 0, (int)((Math.sin(.01 * counter) + 1) * 255 / 2), (int)((-Math.sin(.01 * counter) + 1) * 255 / 2));
            SmartDashboard.putNumber("Green", (int)((Math.sin(.01 * counter) + 1) * 255 / 2));
            SmartDashboard.putNumber("Blue", (int)((-Math.sin(.01 * counter) + 1) * 255 / 2));
            //m_ledBuffer.setRGB(i, 255, 255, 255);
        }
        m_led.setData(m_ledBuffer);


            /*for(int i=0; i<m_ledBuffer.getLength(); i+=2) {
                //sets the LEDs to yellow
                m_ledBuffer.setRGB(i, 0, 25, 25);
            }
    
            for(int i=1; i<m_ledBuffer.getLength(); i+=2) {
                m_ledBuffer.setRGB(i, 0, 50, 0);
            }
        
            for(int i=1; i<m_ledBuffer.getLength(); i+=2) {
                //sets the LEDs to yellow
                m_ledBuffer.setRGB(i, 0, 25, 25);
            }

            for(int i=0; i<m_ledBuffer.getLength(); i+=2) {
                m_ledBuffer.setRGB(i, 0, 50, 0);
            }*/
        
    }
}
    

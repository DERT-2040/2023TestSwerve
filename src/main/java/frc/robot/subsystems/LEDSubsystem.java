package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    AddressableLED m_led1;
    AddressableLEDBuffer m_ledBuffer1;
    int counter;
    boolean voltage;
    boolean isYellow;

    public LEDSubsystem() {
        m_led = new AddressableLED(1);
        m_ledBuffer = new AddressableLEDBuffer(72);
        m_led.setLength(m_ledBuffer.getLength());

        /*m_led1 = new AddressableLED(5);
        m_ledBuffer1 = new AddressableLEDBuffer(37);
        m_led1.setLength(m_ledBuffer.getLength());*/

        isYellow = false;

        setColor(false);
        m_led.start();
        //m_led1.start();
        counter = 0;
        voltage = true;

    }

    public void setColor(boolean yellow) {
        isYellow = yellow;
        if(voltage) {
            if(yellow) {
                for(int i=0; i<m_ledBuffer.getLength(); i++) {
                    //sets the LEDs to yellow
                    m_ledBuffer.setRGB(i, 70, 25, 0);
                }

                /*for(int i=0; i<m_ledBuffer1.getLength(); i++) {
                    //sets the LEDs to yellow
                    m_ledBuffer1.setRGB(i, 70, 25, 0);
                }*/

            } else {
                for(int i=0; i<m_ledBuffer.getLength(); i++) {
                    //sets the LEDs to purple
                    m_ledBuffer.setRGB(i, 75, 0, 45);
                }
                /*for(int i=0; i<m_ledBuffer1.getLength(); i++) {
                    //sets the LEDs to purple
                    m_ledBuffer1.setRGB(i, 75, 0, 45);
                }*/
            }
            m_led.setData(m_ledBuffer);
            //m_led1.setData(m_ledBuffer1);
            SmartDashboard.putBoolean("LED Color", isYellow);
        }
    }

    public void switchColor() {
        setColor(!isYellow);
    }


    public void idlePattern() {
        if(voltage) {
            counter++;
            for(int i=0; i<m_ledBuffer.getLength(); i++) {
                //sets the LEDs to yellow
                
                m_ledBuffer.setRGB(i, 0, (int)( .5 * ((Math.sin(.12 * counter + i * .25) + 1) * 20 / 2 + 35)), (int)((-Math.sin(.12 * counter + i * .25) + 1) * 45 / 2));
            
                //m_ledBuffer.setRGB(i, 255, 255, 255);
            }
            /*for(int i=0; i<m_ledBuffer1.getLength(); i++) {
                //sets the LEDs to yellow
                
                m_ledBuffer1.setRGB(i, 0, (int)( .5 * (Math.sin(.1 * counter + i * .25) + 1) * 255 / 2), (int)((-Math.sin(.1 * counter + i * .25) + 1) * 255 / 2));
                
                //m_ledBuffer.setRGB(i, 255, 255, 255);
            }*/
            m_led.setData(m_ledBuffer);
            //m_led1.setData(m_ledBuffer1);


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


    public void lowVoltage() {
        
        for(int i=0; i<m_ledBuffer.getLength(); i++) {
            //sets the LEDs to red
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        /*for(int i=0; i<m_ledBuffer1.getLength(); i++) {
            //sets the LEDs to red
            m_ledBuffer1.setRGB(i, 255, 0, 0);
        }*/
        m_led.setData(m_ledBuffer);
        //m_led1.setData(m_ledBuffer1);
        voltage = false;
    }

    
    public void resetVoltage() {
        voltage = true;
    }
}
    

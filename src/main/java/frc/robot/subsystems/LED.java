package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final AddressableLED m_led = new AddressableLED(0);
    private final AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(18);
    private LEDPattern red = LEDPattern.solid(Color.kGreen);
    private LEDPattern blue = LEDPattern.solid(Color.kBlue);
    private LEDPattern green = LEDPattern.solid(Color.kRed);
    private LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    private LEDPattern pink = LEDPattern.solid(Color.kTeal);
    
    public LED (){
        m_led.setLength(m_buffer.getLength());
        m_led.setData(m_buffer);
        m_led.start();
    }

    public void SolidGreen(){
        green.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void SolidRed(){
        red.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void SolidBlue(){
        blue.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void SolidYellow(){
        yellow.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void SolidPink(){
        pink.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }
}
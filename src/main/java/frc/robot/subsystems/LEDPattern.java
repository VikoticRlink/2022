package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDPattern {
    private final int kPatternLength = 4;
    private Color m_pattern[];
    private int m_arraySize;  /**length of LED strip */
    private int m_startIndex; /**starting index in the LED strip */
    private int m_iteration;  /**number of times array has been processed */
    private int m_arrayPos;  /**current position to start writing into the LED strip array */
    private int m_patternStart; /**where in the pattern to begin to copy from */
    private int m_patternEnd;  /**where in the pattern to stop copying */

    public enum Direction {kForward, kReverse};
    private Direction m_direction;

    /**
     * Creates an LEDPattern object that will display a pattern
     * in an LED strip
     * @param startIndex  Starting index in the LED array where
     *                    the pattern will be displayed
     * @param numLEDs Number of LEDs in the strip to walk the pattern through
     */
    public LEDPattern(int startIndex, int numLEDs) {
        m_startIndex = startIndex;
        m_arraySize = numLEDs;
        m_direction = Direction.kForward;
        reset();

        // Initialize the LED pattern array
        Color half = new Color(0.0, 0.5, 0.0);
        Color full = new Color(0.0, 1.0, 0.0);
        m_pattern =  new Color[kPatternLength];
        m_pattern[0] = half;
        m_pattern[1] = full;
        m_pattern[2] = full;
        m_pattern[3] = half;
    }

    /** Resets the LED pattern to its initial position in the target
     *  LED strip
    */
    public void reset() {
        m_iteration = 0;
        m_arrayPos = 0;
        m_patternStart = 0;
        m_patternEnd = 0;
    }

    /** Translate an index in the LED array according to whether 
     *  the pattern is being moved forward or reverse in the array
     */
    private int translateIndex(int index) {
        int result = index;
        if (m_direction == Direction.kReverse) {
            result = (m_arraySize - 1) - index;
        }

        return result;
    }


    /** Display's this object's pattern on a given LED array
     * 
     * @param ledBuffer  The LED strip array to display in
     */
    public void process(AddressableLEDBuffer ledBuffer) {
        // Copy the pattern into ledArray starting at the current
        // array index
        int a = m_startIndex + translateIndex(m_arrayPos);
        for (int p = m_patternStart; p <= m_patternEnd; ++p) {
            ledBuffer.setLED(a, m_pattern[p]);
            a += (m_direction == Direction.kForward) ? -1 : 1;
        }

        m_iteration += 1;
        if (m_iteration > (m_arraySize + kPatternLength - 2)) {
            reset();
        }
        else {
            if (m_arrayPos < (m_arraySize - 1)) {
                m_arrayPos += 1;
            }

            if (m_patternStart >= m_arraySize)
            {
                m_patternStart += 1;
            }

            if (m_patternEnd < (kPatternLength - 1)) {
                m_patternEnd += 1;
            }
        }
    }
}

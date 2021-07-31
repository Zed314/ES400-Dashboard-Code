/*
 * tm1620.hpp
 *
 *  Class to pilot TM1620 led driver :
 *
 * On es400 dashboard, TM1620 is connected like so :
 *  - 5 : DIN
 *  - 6 : CLK
 *  - 7 : STB
 */

#ifndef SRC_TM1620_HPP_
#define SRC_TM1620_HPP_



class TM1620
{
public:

    /**
     * Create new TM1620.
     *
     * \param[in] dn Data IN GPIO A pin
     * \param[in] clk Clock GPIO A pin
     * \param[in] stb Standby GPIO A pin
     */
    TM1620(char din, char clk, char stb);

    ~TM1620();

    /**
     * Configure display. Must be done once before using display
     *
     */
    void start_display(void);

    /**
     * Converts number into seven segment, and store it into 7 LSB of out.
     *
     * \param[in] number number to be converted [0,9]
     * \param[out] out converted number
     */
    void convertToSeven(int number, char *out);

    /**
     * Send data to TM1620
     *
     * \param[in] address destination address
     * \param[in] data command to send
     *
     */
    void sendData(char address, char data);

    /**
     * Erase display
     *
     */
    void clearDisplay();


private:


    /**
     * Send command to TM1620
     *
     * \param[in] data command to send
     *
     */
    void send_cmd(char data);

    /**
     * Start of command.
     */
    void start_cmd(void);

    /**
     * End of command.
     *
     */
    void stop_cmd(void);

    /**
     * Send command to TM1620
     *
     * \param[in] cmd command to send
     */
    void sendCommand(char cmd);

    /**
     * Configure display. Must be done once before using display
     *
     */
    void setupDisplay();

    /** Array for number to seven segment conversion */
    const static char conv_seven[];
    /** Data IN GPIO A pin */
    char din;
    /** Clock GPIO A pin */
    char clk;
    /** Standby GPIO A pin */
    char stb;
};

#endif /* SRC_TM1620_HPP_ */

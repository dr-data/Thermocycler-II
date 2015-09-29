#ifndef LCDSCREEN_h
#define LCDSCREEN_h

class LCDScreen
{
    public:
        LCDScreen();
	void init();
        void setup();
	void displayInitialStatus( double temp, int current_cycle  );
	void updateArrow( double temp );
	void finalMess();	
	int retrieveTF();
	int retrieveTC();
	int retrieveMC();
	void printError( int mess );

    private:
	void waitForSelect();
	void setUserInputs();
	void selectHighLowPush();
	void selectCycles();
	void dispUserPrefHighLow();
	void dispUserPrefCycles();
	
};

#endif

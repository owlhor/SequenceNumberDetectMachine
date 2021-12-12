/* FRA221 Digital Electronics Competency project
 * Nucleo code detector
 * detect code 60%32 = 28
 * use NUCLEO-F411RE with SN74hc166, sn74hc595 
 * written on mbed compiler online by owl_hor, FRAB7 FIBO KMUTT, Dec 2021.
 */

#include "mbed.h"
#include "platform/mbed_thread.h"


DigitalIn I0(A0); // d bounce count
DigitalIn C1(A1); // switch code in 
DigitalOut  LatchCLK(D6),SER(D7);              // 595 LED
DigitalOut SHLD(D2),SERCLK(D3),INHCLK(D4),LATCHER(D5);  // 166x595 7 segment
BusOut sevseg(D9, D10, D11, D12, D13, D14, D15); // 7 segment out bus
DigitalOut  crctr(D8),start_ld(D0);            // debugger
uint8_t cno = 0b0,cnock = 0b0,enbounce = 0;    //input code for check
uint8_t sevdg = 0,sevcode=0b00000000;          // special drive 28_
uint32_t TStamp =0,DishTime=0;                 // time special drive 28_
uint32_t timeStampSR =0;                       // clock loop
void sevenSegmentDriverLoop(uint8_t I8bitIn);  // 595 driver
void ParraRegisDrive(uint8_t inp);             // 166x595 driver
void CodeShifter();                            // shift when bounce clock
void STATUMDetect();                           // Grand State Machine
void SegSpecialOrder();                        // 166x595 special drive 28_

int fallingEdgeDetect()
{
    static uint8_t buttonstate = 0;
    uint8_t Res = -1;//Default Result, mean nothing change
    if (buttonstate == 0) {
        if (I0) {//if last state is low & now state is high
            Res = 1; // Rising
            buttonstate = 1;
        }
    }
    else {
        if ((!I0)) {///if last state is High & now state is low
            buttonstate = 0;
            Res = 0; // falling
                    }
        } return Res;
}


int main()
{
while(true)
    {
        uint32_t time = us_ticker_read();
        if(time-timeStampSR > 1000)       // run every 1000us 
        {
            timeStampSR = time;           //set new time stamp
            sevenSegmentDriverLoop(cno);  //call 74hc595 management function
            STATUMDetect();               // state machine
        }

        int8_t bounce = fallingEdgeDetect();
        if(bounce == 1) // time-timeStampDigit > 250000 change output
        {
            enbounce++;                   // check START state(no bounce before) 
            CodeShifter();
        }
            
    }   //end while
}       // end int main

void sevenSegmentDriverLoop(uint8_t I8bitIn)
{
    static enum {INIT,HSRCLK,LSRCLK,HRCLK} SRState = INIT;
    static uint8_t SegPosition =0;
    switch(SRState)
    {
        case INIT: //init all pin that use to send data
        default:
        SERCLK = 0; 
        LatchCLK = 0;
        SER = 0;
        SegPosition =0;
        SRState = LSRCLK;  // go to Latcher
        break;

        case LSRCLK: // change SER output
        SERCLK = 0;  // SRCLK
        LatchCLK = 0;
        SER = (I8bitIn>>SegPosition)&0x01; //call data in each bit
        SRState = HSRCLK;
        break;

        case HSRCLK: // rising SRCLK to sent data to 74hc595
        SERCLK = 1;  // SRCLK
        LatchCLK = 0;
        SegPosition++;

        if(SegPosition>=8)
        {
            SRState = HRCLK; // go to latch
        }
        else
        {
            SRState = LSRCLK;
        }
        break;

        case HRCLK: //rising LatchCLK to output new 8 parallel data in 74hc595
        SERCLK = 0;  // SRCLK
        LatchCLK = 1;
        
        SRState = INIT; // finish
        break;

        
    }// end switch
}// end void

void CodeShifter()
{
    cno = (cno << 1)+ C1; // shift old and add new
    cnock = cno << 3;     // for condition check

}// end void

void STATUMDetect(){     // Grand STATE MACHINE  for system
    static enum {START,CHECK,MATCH} STATUM = START;
    switch (STATUM){
            case START:
            default:
            start_ld = 1;
            crctr= 0 ;
            ParraRegisDrive(0b01011011);// 7segment = 0b0101 1011 S,5
            
            switch (enbounce){  
                case not 0:     // bounce not 0 times mean input recieved
                STATUM = CHECK ; 
                break; }
            break;
            
            case CHECK:
            start_ld = 0;
            crctr= 0;
            ParraRegisDrive(0b01001110); // 7segment = 0b01001110 C
                switch(cnock){
                    case 0b11100000:     // match with 28(11100 << 3)
                        STATUM = MATCH;
                    break;}
            break;
            
            case MATCH:
            start_ld = 0;
            crctr = 1;
            SegSpecialOrder();        // generate  special 2 8 _ in savcode
            ParraRegisDrive(sevcode); //
            if(cnock != 0b11100000){  // back to start after match
                cno = 0b0; cnock = 0b0,enbounce = 0;
                STATUM = START ;
                }
            break;
            } //end switch statum
}// end void STATUM

void SegSpecialOrder()// 7 segment special 2 8 _ 0b01101101 0b01111111 0b00000000
{
    uint32_t timer = us_ticker_read();
    if(timer-DishTime > 1000000) // change output every 1000 msec
        {
            DishTime =timer;
            if (sevdg >= 3){
                sevdg = 0; }   //reset
            else{ sevdg++; }
        }
        switch (sevdg){
            case 0:
            default: 
            sevcode = 0b00001000; break; //_ 
            
            case 1:
            sevcode = 0b01101101; break; //2
            
            case 2:
            sevcode = 0b01111111; break; //8
            }//end switch
    } // end void special segment
    
void ParraRegisDrive(uint8_t inp){   // sn74hc166x595 drive
    static enum {INIT,SHLDER,SFTER,RESFTER,LATCHING} PRState = INIT;
    static uint8_t SegDigit = 0; // count digits that serial data sent
    switch (PRState)
    {
        case INIT: // preparation
        default:
        SHLD = 1; //bar disable
        SERCLK = 0;
        INHCLK = 0;
        LATCHER = 0;
        
        SegDigit = 0;
        sevseg = inp; // get "inp" parallel out
        PRState = SHLDER; // go to latcher after preparation
        break;
        
        case SHLDER:  //7seg = inp; // get parallel out in i66
        sevseg = inp;
        SHLD = 0;     // trig parallel get 1 times
        SERCLK = 0;
        INHCLK = 1;   // use inhibit clk to trig / no distrub ser clk
        LATCHER = 0;
        
        PRState = SFTER; 
        break; 

        case SFTER: // send serial
        SHLD = 1;
        SERCLK = 1;
        INHCLK = 0;
        LATCHER = 0;
        
        SegDigit++;
        if(SegDigit >= 9){
            PRState = LATCHING; // finish and reshift next dataset
            }
        else{
            PRState = RESFTER; // down clock to shift next serial digit
            }
        break;
        
        case RESFTER: // down clock before send next digit
        SHLD = 1;
        SERCLK = 0; // down clock
        INHCLK = 0;
        LATCHER = 0;
        
        PRState = SFTER;
        break;
        
        case LATCHING:  // latch for 595
        SHLD = 1;
        SERCLK = 0; // down clock
        INHCLK = 0;
        LATCHER = 1;
        
        PRState = INIT;
        break;
        
        }//end switch
    }//end void prll
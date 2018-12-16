#include <stm32.h>
#include <gpio.h>
#include <delay.h>
#include <string.h>
#include <irq.h>
#include <stdio.h>

#define HSI_HZ 16000000U
#define PCLK1_HZ HSI_HZ

#define I2C_SPEED_HZ 100000
#define PCLK1_MHZ 16

#define CTRL_REG1 0x20
#define OUT_X 0x29
#define OUT_Y 0x2B
#define OUT_Z 0x2D
#define LIS35DE_ADDR 0x1D

#define SEND_BUFFER_SIZE 200

#define MAX_WAIT_VALUE 40000

char sendBuffer[SEND_BUFFER_SIZE];
int sendBufferPosition = 0;
int charsToSend = 0;

char tempBuffer[SEND_BUFFER_SIZE];

volatile int gyroReady = 0;

int wasDataSend = 0;
int wasRepeatedStart = 0;

volatile irq_level_t currentLevel;

void intToChar(char* buffer, int value, int offset) {
    for (int j = offset; value != 0 && j > 0; j--) {
        buffer[j] = (value % 10) + '0';
        value /= 10;
    }
}

void sendToBuffer(const char* command) {
    currentLevel = IRQprotectAll();

    for (int j = 0; command[j] != '\r' ;j++) {
        if (sendBufferPosition >= SEND_BUFFER_SIZE) {
            IRQunprotectAll(currentLevel);
            return;
        }

        sendBuffer[sendBufferPosition] = command[j];
        sendBufferPosition++;
        charsToSend++;
    }

    if (sendBufferPosition >= SEND_BUFFER_SIZE) {
        IRQunprotectAll(currentLevel);
        return;
    }


    sendBuffer[sendBufferPosition] = '\r';
    sendBufferPosition++;
    charsToSend++;

    IRQunprotectAll(currentLevel);
}

void sendBufferToDMA() {
    //check if we can send to DMA
    currentLevel = IRQprotectAll();
    int wasDMABusy = 0;
    if (charsToSend > 0) {
        if ((DMA1_Stream6->CR & DMA_SxCR_EN) == 0 && (DMA1->HISR & DMA_HISR_TCIF6) == 0) {
            int tempBufferSize = charsToSend;

            for (int i = 0; i < tempBufferSize; i++) {
                tempBuffer[i] = sendBuffer[i];
            }

            DMA1_Stream6->M0AR = (uint32_t) tempBuffer;
            DMA1_Stream6->NDTR = tempBufferSize;
            DMA1_Stream6->CR |= DMA_SxCR_EN;

            sendBufferPosition = 0;
            charsToSend = 0;
        } else {
            wasDMABusy = 1;
        }
    }
    IRQunprotectAll(currentLevel);

    if (wasDMABusy) {
//        sendToBuffer("DMA BUSY \n\r");
    }
}


void DMA1_Stream6_IRQHandler() {
    uint32_t isr = DMA1->HISR;
    if (isr & DMA_HISR_TCIF6) {

        DMA1->HIFCR = DMA_HIFCR_CTCIF6;

        sendBufferToDMA();
    }
}

void configureUsartAndDma() {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOafConfigure(GPIOA,
                    2,
                    GPIO_OType_PP,
                    GPIO_Fast_Speed,
                    GPIO_PuPd_NOPULL,
                    GPIO_AF_USART2);

    GPIOafConfigure(GPIOA,
                    3,
                    GPIO_OType_PP,
                    GPIO_Fast_Speed,
                    GPIO_PuPd_UP,
                    GPIO_AF_USART2);

    USART2->CR1 = USART_CR1_RE | USART_CR1_TE;

    USART2->CR2 = 0;

    uint32_t const baudrate = 9600U;
    USART2->BRR = (PCLK1_HZ + (baudrate / 2U)) / baudrate;

    USART2->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;

    DMA1_Stream6->CR = 4U << 25       |
                       DMA_SxCR_PL_1  |
                       DMA_SxCR_MINC  |
                       DMA_SxCR_DIR_0 |
                       DMA_SxCR_TCIE;

    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;

    DMA1->HIFCR = DMA_HIFCR_CTCIF6;

    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    USART2->CR1 |= USART_CR1_UE;

}

void configureGyro() {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOafConfigure(GPIOB, 8, GPIO_OType_OD,
                    GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                    GPIO_AF_I2C1);
    GPIOafConfigure(GPIOB, 9, GPIO_OType_OD,
                    GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                    GPIO_AF_I2C1);

    I2C1->CR1 = 0;

    I2C1->CR2 = PCLK1_MHZ;
    I2C1->CCR = (PCLK1_MHZ * 1000000) /
                (I2C_SPEED_HZ << 1);
    I2C1->TRISE = PCLK1_MHZ + 1;

    I2C1->CR1 |= I2C_CR1_PE;

    //przerwania
    I2C1->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

}

void saveValueToRegister() {
    //Zainicjuj transmisję sygnału START
    I2C1->CR1 |= I2C_CR1_START;

    /*
    //Czekaj na zakończenie transmisji bitu START, co jest
    //sygnalizowane ustawieniem bitu SB (ang.start bit) w rejestrze SR1,
    //czyli czekaj na spełnienie warunku
    int i = 0;
    while (!(I2C1->SR1 & I2C_SR1_SB) && i <= MAX_WAIT_VALUE) {
        __NOP();
        i++;
    }
    if (i >= MAX_WAIT_VALUE) {
        sendToBuffer("cond 1 failed\n\r");
        sendBufferToDMA();
    }

    //Zainicjuj wysyłanie 7-bitowego adresu slave’a, tryb MT
    I2C1->DR = LIS35DE_ADDR << 1;

    //Czekaj na zakończenie transmisji adresu, ustawienie bitu ADDR (ang.address sent)
    // w rejestrze SR1, warunek
    i = 0;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && i <= MAX_WAIT_VALUE) {
        __NOP();
        i++;
    }
    if (i >= MAX_WAIT_VALUE) {
        sendToBuffer("cond 2 failed\n\r");
        sendBufferToDMA();
    }

    //Skasuj bit ADDR przez odczytanie rejestru SR2 po odczytaniu rejestru SR1
    I2C1->SR2;

    //Zainicjuj wysyłanie 8-bitowego numeru rejestru slave’a
    I2C1->DR = CTRL_REG1;

    //Czekaj na opróżnienie kolejki nadawczej, czyli na ustawienie bitu TXE
    // (ang. transmitter data register empty) w rejestrze SR1, warunek
    i = 0;
    while (!(I2C1->SR1 & I2C_SR1_TXE) && i <= MAX_WAIT_VALUE) {
        __NOP();
    }
    if (i >= MAX_WAIT_VALUE) {
        sendToBuffer("cond 3 failed\n\r");
        sendBufferToDMA();
    }

    uint32_t powerUp = 1 << 6;
    uint32_t zEnabled = 1 << 2;
    uint32_t yEnabled = 1 << 1;
    uint32_t xEnabled = 1;
    uint32_t reqValue =  (zEnabled | yEnabled | xEnabled | powerUp);

    //Wstaw do kolejki nadawczej 8-bitową wartość zapisywaną do rejestru slave’a
    I2C1->DR = reqValue;

    //Czekaj na zakończenie transmisji, czyli na ustawienie bitu BTF
    // (ang. byte transfer finished) w rejestrze SR1, warunek
    i = 0;
    while (!(I2C1->SR1 & I2C_SR1_BTF) && i <= MAX_WAIT_VALUE) {
        __NOP();
    }
    if (i >= MAX_WAIT_VALUE) {
        sendToBuffer("cond 4 failed\n\r");
        sendBufferToDMA();
    }

    //Zainicjuj transmisję sygnału STOP
    I2C1->CR1 |= I2C_CR1_STOP;
     */
}

void readFromGyro() {
    //Zainicjuj transmisję sygnału START
    I2C1->CR1 |= I2C_CR1_START;

    /*
    //Czekaj na ustawienie bituSBw rejestrze SR1, warunek
    int i = 0;
    while (!(I2C1->SR1 & I2C_SR1_SB) && i <= MAX_WAIT_VALUE) {
        __NOP();
        i++;
    }
    if (i >= MAX_WAIT_VALUE) {
        sendToBuffer("cond 5 failed\n\r");
        sendBufferToDMA();
    }

    //Zainicjuj wysyłanie 7-bitowego adresu slave’a, tryb MT
    I2C1->DR = LIS35DE_ADDR << 1;

    //Czekaj na zakończenie transmisji adresu, warunek
    i = 0;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && i <= MAX_WAIT_VALUE) {
        __NOP();
        i++;
    }
    if (i >= MAX_WAIT_VALUE) {
        sendToBuffer("cond 6 failed\n\r");
        sendBufferToDMA();
    }

    //Skasuj bitADDR
    I2C1->SR2;

    //Zainicjuj wysyłanie numeru rejestru slave’a
    I2C1->DR = OUT_Y;

    //Czekaj na zakończenie transmisji, czyli na ustawienie bitu BTF (ang. byte transfer finished) w rejestrze SR1,
    // czyli na spełnienie warunku
    i = 0;
    while (!(I2C1->SR1 & I2C_SR1_BTF) && i <= MAX_WAIT_VALUE) {
        __NOP();
        i++;
    }
    if (i >= MAX_WAIT_VALUE) {
        sendToBuffer("cond 7 failed\n\r");
        sendBufferToDMA();
    }

    //Zainicjuj transmisję sygnału REPEATED START
    I2C1->CR1 |= I2C_CR1_START;

    //Czekaj na ustawienie bitu SB w rejestrze SR1, warunek
    i = 0;
    while (!(I2C1->SR1 & I2C_SR1_SB) && i <= MAX_WAIT_VALUE) {
        __NOP();
        i++;
    }
    if (i >= MAX_WAIT_VALUE) {
        sendToBuffer("cond 8 failed\n\r");
        sendBufferToDMA();
    }

    //Zainicjuj wysyłanie 7-bitowego adresu slave’a, tryb MR
    I2C1->DR = (LIS35DE_ADDR << 1) | 1U;


    //Ustaw, czy po odebraniu pierwszego bajtu ma być wysłany sygnał ACK czy NACK
    //Ponieważ ma być odebrany tylko jeden bajt, ustaw wysłanie sygnału NACK, zerując bit ACK
    I2C1->CR1 &= ~I2C_CR1_ACK;

    //Czekaj na zakończenie transmisji adresu, warunek
    i = 0;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && i <= MAX_WAIT_VALUE) {
        __NOP();
        i++;
    }
    if (i >= MAX_WAIT_VALUE) {
        sendToBuffer("cond 9 failed\n\r");
        sendBufferToDMA();
    }

    //Skasuj bit ADDR
    I2C1->SR2;

    //Zainicjuj transmisję sygnału STOP, aby został wysłany po
    //odebraniu ostatniego (w tym przypadku jedynego) bajtu
    I2C1->CR1 |= I2C_CR1_STOP;

    //Czekaj na ustawienie bitu RXNE (ang. receiver data register not empty) w rejestrze SR1, warunek
    i = 0;
    while (!(I2C1->SR1 & I2C_SR1_RXNE) && i <= MAX_WAIT_VALUE) {
        __NOP();
        i++;
    }
    if (i >= MAX_WAIT_VALUE) {
        sendToBuffer("cond 10 failed\n\r");
        sendBufferToDMA();
    }

    //Odczytaj odebraną 8-bitową wartość
    int value = I2C1->DR;

    return value;
    */
}

int firstBtf = 1;


void I2C1_EV_IRQHandler(void) {
    if (gyroReady) {
        if (I2C1->SR1 & I2C_SR1_SB) {
            if (!wasRepeatedStart) {
                //1
                sendToBuffer("I2C_SR1_SB 1\n\r");
                sendBufferToDMA();

                //Zainicjuj wysyłanie 7-bitowego adresu slave’a, tryb MT
                I2C1->DR = LIS35DE_ADDR << 1;
            }  else {
                //4
                sendToBuffer("I2C_SR1_SB 4\n\r");
                sendBufferToDMA();

                //Zainicjuj wysyłanie 7-bitowego adresu slave’a, tryb MR
                I2C1->DR = (LIS35DE_ADDR << 1) | 1U;


                //Ustaw, czy po odebraniu pierwszego bajtu ma być wysłany sygnał ACK czy NACK
                //Ponieważ ma być odebrany tylko jeden bajt, ustaw wysłanie sygnału NACK, zerując bit ACK
                I2C1->CR1 &= ~I2C_CR1_ACK;
            }

        }

        if (I2C1->SR1 & I2C_SR1_ADDR) {
            if (!wasRepeatedStart) {
                //2
                sendToBuffer("I2C_SR1_ADDR 2\n\r");
                sendBufferToDMA();

                //Skasuj bit ADDR przez odczytanie rejestru SR2 po odczytaniu rejestru SR1
                I2C1->SR2;

                //Zainicjuj wysyłanie numeru rejestru slave’a
                I2C1->DR = OUT_Y;
            } else {
                //5
                sendToBuffer("I2C_SR1_ADDR 5\n\r");
                sendBufferToDMA();

                I2C1->CR2 |= I2C_CR2_ITBUFEN;

                //Skasuj bit ADDR
                I2C1->SR2;

                //Zainicjuj transmisję sygnału STOP, aby został wysłany po
                //odebraniu ostatniego (w tym przypadku jedynego) bajtu
                I2C1->CR1 |= I2C_CR1_STOP;
            }
        }


        if (I2C1->SR1 & I2C_SR1_BTF) {
            //3
            sendToBuffer("I2C_SR1_BTF 3\n\r");
            sendBufferToDMA();

            if (firstBtf) {
                wasRepeatedStart = 1;

                //Zainicjuj transmisję sygnału REPEATED START
                I2C1->CR1 |= I2C_CR1_START;
            } else {

            }

        }

        if (I2C1->SR1 & I2C_SR1_RXNE) {
            //6
            sendToBuffer("I2C_SR1_RXNE 6\n\r");
            sendBufferToDMA();

            //Odczytaj odebraną 8-bitową wartość
            int value = I2C1->DR;

            if (value != 0) {
                char num[32];
                for (int j = 0; j < 32; j++) {
                    num[j] = 0;
                }

                intToChar(num, value, 29);

                num[30] = '\n';
                num[31] = '\r';

                sendToBuffer(num);

                sendToBuffer("dziala 1\n\r");
            } else {
                sendToBuffer("dziala 2\n\r");
            }
            sendBufferToDMA();
        }


    } else {
        int samePass = 0;
        if (I2C1->SR1 & I2C_SR1_SB) {

            sendToBuffer("I2C_SR1_SB\n\r");
            sendBufferToDMA();

            //Zainicjuj wysyłanie 7-bitowego adresu slave’a, tryb MT
            I2C1->DR = LIS35DE_ADDR << 1;

        }

        if (I2C1->SR1 & I2C_SR1_ADDR) {
            sendToBuffer("I2C_SR1_ADDR\n\r");
            sendBufferToDMA();

            samePass = 1;

            I2C1->CR2 |= I2C_CR2_ITBUFEN;
            wasDataSend = 1;

            //Skasuj bit ADDR przez odczytanie rejestru SR2 po odczytaniu rejestru SR1
            I2C1->SR2;

            //Zainicjuj wysyłanie 8-bitowego numeru rejestru slave’a
            I2C1->DR = CTRL_REG1;
        }

        if ((I2C1->SR1 & I2C_SR1_TXE) && wasDataSend && !samePass) {
            sendToBuffer("I2C_SR1_TXE\n\r");
            sendBufferToDMA();

            I2C1->CR2 &= (~((uint32_t)0x00000000)) ^ I2C_CR2_ITBUFEN;
            wasDataSend = 0;

            uint32_t powerUp = 1 << 6;
            uint32_t zEnabled = 1 << 2;
            uint32_t yEnabled = 1 << 1;
            uint32_t xEnabled = 1;
            uint32_t reqValue =  (zEnabled | yEnabled | xEnabled | powerUp);

            //Wstaw do kolejki nadawczej 8-bitową wartość zapisywaną do rejestru slave’a
            I2C1->DR = reqValue;
        }

        if (I2C1->SR1 & I2C_SR1_BTF) {
            sendToBuffer("I2C_SR1_BTF\n\r");
            sendBufferToDMA();

            if (firstBtf) {
                firstBtf = 0;
                I2C1->CR1 |= I2C_CR1_STOP;
            } else {
                gyroReady = 1;
                firstBtf = 1;

                int i = 0;
                while (i < 10000000) {
                    __NOP();
                    i++;
                }

                readFromGyro();
            }
        }
    }

}

void I2C1_ER_IRQHandler(void) {

    sendToBuffer("Error at I2C occur\n\r");
    sendBufferToDMA();
}

void configureCounter() {
    //Włączenie taktowania układu licznika
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    //zliczaj w gore
    TIM3->CR1 = 0;

    //Skonfigurowanie preskalera i zakresu zliczania
    TIM3->PSC = 20;
    TIM3->ARR = 3200000000;//10s

    //wymuszenie zdarzenia uaktualnienia
    TIM3->EGR = TIM_EGR_UG;

    //Uruchomienie licznika
    TIM3->CR1 |= TIM_CR1_CEN;

    //wlaczenie przerwania
    TIM3->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF);
    TIM3->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;

    NVIC_EnableIRQ(TIM3_IRQn);


}

void TIM3_IRQHandler(void) {
    uint32_t it_status = TIM3->SR & TIM3->DIER;
    if (it_status & TIM_SR_UIF) {
        TIM3->SR = ~TIM_SR_UIF;
        //uaktualnienie
    }
    if (it_status & TIM_SR_CC1IF) {
        TIM3->SR = ~TIM_SR_CC1IF;
//        sendToBuffer("licznik\n\r");
//        sendBufferToDMA();
    }
}


int main() {
    //gpio enable
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    //interrupts enable
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    configureUsartAndDma();

    configureGyro();

//    configureCounter();

    sendToBuffer("break 1\n\r");
    sendBufferToDMA();

    sendToBuffer("break 2\n\r");
    sendBufferToDMA();

    saveValueToRegister();

    sendToBuffer("break 3\n\r");
    sendBufferToDMA();

    __NOP();

    int i = 0;


    while (1) {
        if (!gyroReady) {
            __NOP();
            continue;
        }

        i++;
        if (i > 10000000) {

//            sendToBuffer("Start reading\n\r");
//            sendBufferToDMA();

//            readFromGyro();
            /*
            int value = readFromGyro();


            if (value != 0) {
                char num[32];
                for (int j = 0; j < 32; j++) {
                    num[j] = 0;
                }

                intToChar(num, value, 29);

                num[30] = '\n';
                num[31] = '\r';

//                sendToBuffer("dziala\n\r");
                sendToBuffer(num);
            } else {
//                char num[32];
//                intToStr(12345, num);

//                int x = value;
//                for (int j = 0; x != 0; j++) {
//                    num[j] = (x % 10) + '0';
//                    x /= 10;
//                }
//
//                num[30] = '\n';
//                num[31] = '\r';
//
//                char buf[3] = "3\n\r";
//
//                sendToBuffer(num);
                sendToBuffer("nie dziala\n\r");
//
            }

            */

//            sendBufferToDMA();
//            i = 0;
        }

    }
}

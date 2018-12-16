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

#define SEND_BUFFER_SIZE 400

#define MAX_WAIT_VALUE 40000

int debug = 0;

char sendBuffer[SEND_BUFFER_SIZE];
int sendBufferPosition = 0;
int charsToSend = 0;

char tempBuffer[SEND_BUFFER_SIZE];

volatile int accelerometerReady = 0;

int dataWasSend = 0;
int firstBtf = 1;

int accelerationReadPending = 0;
int readXAxis = 0;
int xAxisAcceleration = 0;
int yAxisAcceleration = 0;
int receivePhase = 0;

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

void configureAccelerometer() {
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

void sendAccelerationToUART() {
    char num[32];
    for (int j = 0; j < 32; j++) {
        num[j] = 0;
    }

    intToChar(num, xAxisAcceleration, 29);
    intToChar(num, yAxisAcceleration, 25);

    num[26] = ',';

    num[30] = '\n';
    num[31] = '\r';

    sendToBuffer(num);
    sendBufferToDMA();
}

void saveConfigurationToAccelerometer() {
    dataWasSend = 0;
    firstBtf = 1;
    I2C1->CR1 |= I2C_CR1_START;
}

void readXAxisAcceleration() {
    readXAxis = 1;
    accelerationReadPending = 1;
    receivePhase = 0;

    I2C1->CR1 |= I2C_CR1_START;
}

void readYAxisAcceleration() {
    readXAxis = 0;
    accelerationReadPending = 1;
    receivePhase = 0;

    I2C1->CR1 |= I2C_CR1_START;
}

void handleAccelerometerSetup() {
    if (I2C1->SR1 & I2C_SR1_SB) {
        if (debug) {
            sendToBuffer("I2C_SR1_SB\n\r");
            sendBufferToDMA();
        }

        //Zainicjuj wysyłanie 7-bitowego adresu slave’a, tryb MT
        I2C1->DR = LIS35DE_ADDR << 1;
    }

    if (I2C1->SR1 & I2C_SR1_ADDR) {
        if (debug) {
            sendToBuffer("I2C_SR1_ADDR\n\r");
            sendBufferToDMA();
        }

        I2C1->CR2 |= I2C_CR2_ITBUFEN;
        dataWasSend = 1;

        //Skasuj bit ADDR przez odczytanie rejestru SR2 po odczytaniu rejestru SR1
        I2C1->SR2;

        //Zainicjuj wysyłanie 8-bitowego numeru rejestru slave’a
        I2C1->DR = CTRL_REG1;
    }

    if ((I2C1->SR1 & I2C_SR1_TXE) && dataWasSend) {
        if (debug) {
            sendToBuffer("I2C_SR1_TXE\n\r");
            sendBufferToDMA();
        }

        I2C1->CR2 &= (~((uint32_t)0x00000000)) ^ I2C_CR2_ITBUFEN;
        dataWasSend = 0;

        uint32_t powerUp = 1 << 6;
        uint32_t zEnabled = 1 << 2;
        uint32_t yEnabled = 1 << 1;
        uint32_t xEnabled = 1;
        uint32_t reqValue =  (zEnabled | yEnabled | xEnabled | powerUp);

        //Wstaw do kolejki nadawczej 8-bitową wartość zapisywaną do rejestru slave’a
        I2C1->DR = reqValue;
    }

    if (I2C1->SR1 & I2C_SR1_BTF) {
        if (debug) {
            sendToBuffer("I2C_SR1_BTF\n\r");
            sendBufferToDMA();
        }

        if (firstBtf) {
            firstBtf = 0;
            I2C1->CR1 |= I2C_CR1_STOP;
        } else {
            accelerometerReady = 1;
            firstBtf = 1;
        }
    }
}

void handleAccelerometerRead() {
    if (I2C1->SR1 & I2C_SR1_SB) {
        if (!receivePhase) {
            //1
            if (debug) {
                sendToBuffer("I2C_SR1_SB 1\n\r");
                sendBufferToDMA();
            }

            //Zainicjuj wysyłanie 7-bitowego adresu slave’a, tryb MT
            I2C1->DR = LIS35DE_ADDR << 1;
        }  else {
            //4
            if (debug) {
                sendToBuffer("I2C_SR1_SB 4\n\r");
                sendBufferToDMA();
            }

            //Zainicjuj wysyłanie 7-bitowego adresu slave’a, tryb MR
            I2C1->DR = (LIS35DE_ADDR << 1) | 1U;

            //Ustaw, czy po odebraniu pierwszego bajtu ma być wysłany sygnał ACK czy NACK
            //Ponieważ ma być odebrany tylko jeden bajt, ustaw wysłanie sygnału NACK, zerując bit ACK
            I2C1->CR1 &= ~I2C_CR1_ACK;
        }

    }

    if (I2C1->SR1 & I2C_SR1_ADDR) {
        if (!receivePhase) {
            //2
            if (debug) {
                sendToBuffer("I2C_SR1_ADDR 2\n\r");
                sendBufferToDMA();
            }

            //Skasuj bit ADDR przez odczytanie rejestru SR2 po odczytaniu rejestru SR1
            I2C1->SR2;

            //Zainicjuj wysyłanie numeru rejestru slave’a

            if (readXAxis) {
                I2C1->DR = OUT_X;
            } else {
                I2C1->DR = OUT_Y;
            }
        } else {
            //5
            if (debug) {
                sendToBuffer("I2C_SR1_ADDR 5\n\r");
                sendBufferToDMA();
            }

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
        if (debug) {
            sendToBuffer("I2C_SR1_BTF 3\n\r");
            sendBufferToDMA();
        }

        receivePhase = 1;

        //Zainicjuj transmisję sygnału REPEATED START
        I2C1->CR1 |= I2C_CR1_START;

    }

    if (I2C1->SR1 & I2C_SR1_RXNE) {
        //6
        if (debug) {
            sendToBuffer("I2C_SR1_RXNE 6\n\r");
            sendBufferToDMA();
        }

        //Odczytaj odebraną 8-bitową wartość
        if (readXAxis) {
            xAxisAcceleration = I2C1->DR;
        } else {
            yAxisAcceleration = I2C1->DR;
        }

        if (readXAxis) {
            readYAxisAcceleration();
        } else {
            accelerationReadPending = 0;
            sendAccelerationToUART();
        }
    }

}


void I2C1_EV_IRQHandler(void) {
    if (accelerometerReady) {
        handleAccelerometerRead();
    } else {
        handleAccelerometerSetup();
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
        if (accelerometerReady && !accelerationReadPending) {
            readXAxisAcceleration();
        }
    }
}


int main() {
    //gpio enable
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    //interrupts enable
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    configureUsartAndDma();

    configureAccelerometer();

    saveConfigurationToAccelerometer();

    configureCounter();

    while (1) {
        __NOP();
    }
}

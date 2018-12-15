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

volatile irq_level_t currentLevel;

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

}

void saveValueToRegister() {
    //Zainicjuj transmisję sygnału START
    I2C1->CR1 |= I2C_CR1_START;

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
}

int readFromGyro() {
    //Zainicjuj transmisję sygnału START
    I2C1->CR1 |= I2C_CR1_START;

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
}


int main() {
    //gpio enable
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    //interrupts enable
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    configureUsartAndDma();

    sendToBuffer("break 1\n\r");
    sendBufferToDMA();

    configureGyro();

    sendToBuffer("break 2\n\r");
    sendBufferToDMA();

    saveValueToRegister();

    sendToBuffer("break 3\n\r");
    sendBufferToDMA();

    __NOP();

    int i = 0;

    while (1) {
        i++;
        if (i > INT32_MAX - 1) {
            int value = readFromGyro();

            if (value != 0) {

                char num[32];

                for (int j = 0; j < 32; j++) {
                    num[j] = 0;
                }

                int x = value;
                for (int j = 29; x != 0 && j > 0; j--) {
                    num[j] = (x % 10) + '0';
                    x /= 10;
                }

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
                sendToBuffer("0\n\r");
//
            }


            sendBufferToDMA();
            i = 0;
        }

    }
}

// I'll be using fixed-size types
#include <stdint.h>
#include <stdio.h>
#include <sys/unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

// Include the systick header
#include <libopencm3/cm3/systick.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>

extern "C" {
    #include <hxcmod.h>
}

#include "sober_wednesday.h"

#define BUFFER_SIZE 4096

// Important note for C++: in order for interrupt routines to work, they MUST
// have the exact name expected by the interrupt framework. However, C++ will
// by default mangle names (to allow for function overloading, etc) and so if
// left to it's own devices will cause your ISR to never be called. To force it
// to use the correct name, we must declare the function inside an 'extern C'
// block, as below.
// For more details, see https://en.cppreference.com/w/cpp/language/language_linkage
extern "C" {
    void sys_tick_handler(void);
    ssize_t _write(int file, const char *ptr, ssize_t len);
}

// Storage for our monotonic system clock.
// Note that it needs to be volatile since we're modifying it from an interrupt.
static volatile uint64_t _millis = 0;
static volatile uint32_t idx;

modcontext mctx;

msample data[BUFFER_SIZE]  __attribute__ ((aligned (4)));
tracker_buffer_state mod_state;

int buf;

int _write(int file, const char *ptr, ssize_t len) {
    // If the target file isn't stdout/stderr, then return an error
    // since we don't _actually_ support file handles
    if (file != STDOUT_FILENO && file != STDERR_FILENO) {
        // Set the errno code (requires errno.h)
        errno = EIO;
        return -1;
    }

    // Keep i defined outside the loop so we can return it
    int i;
    for (i = 0; i < len; i++) {
        // If we get a newline character, also be sure to send the carriage
        // return character first, otherwise the serial console may not
        // actually return to the left.
        if (ptr[i] == '\n') {
            usart_send_blocking(USART1, '\r');
        }

        // Write the character to send to the USART1 transmit buffer, and block
        // until it has been sent.
        usart_send_blocking(USART1, ptr[i]);
    }

    // Return the number of bytes we sent
    return i;
}


static void timer_setup(){

	/* Set timer prescaler. 72MHz/1440 => 50000 counts per second. */
	TIM_PSC(TIM1) = 0;

	/* End timer value. If this is reached an interrupt is generated. */
	TIM_ARR(TIM1) = 1633;

	/* Enable timer update DMA request*/
	TIM_DIER(TIM1) |= TIM_DIER_UDE;

	/* Start timer. */
	TIM_CR1(TIM1) |= TIM_CR1_CEN;


}

void dma1_channel5_isr(){

    // Transfer complete: fill the second half of the buffer while playing the first half
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL5, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL5, DMA_TCIF);
        hxcmod_fillbuffer(&mctx,(msample*)&data[BUFFER_SIZE/2], BUFFER_SIZE/2, &mod_state);
    } else if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL5, DMA_HTIF)){
        // Transfer half complete : fill the first half of the buffer while playing the second half
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL5, DMA_HTIF);
        hxcmod_fillbuffer(&mctx, (msample*)&data, BUFFER_SIZE/2, &mod_state);
    }

}

static void dma_setup(){

    // In order to use SPI2_TX, we need DMA 1 Channel 5
    dma_channel_reset(DMA1, DMA_CHANNEL5);

    // SPI2 data register as output
    dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (uint32_t)&TIM3_CCR1);

    // We will be using system memory as the source data
    dma_set_read_from_memory(DMA1, DMA_CHANNEL5);

    // Memory increment mode needs to be turned on, so that if we're sending
    // multiple bytes the DMA controller actually sends a series of bytes,
    // instead of the same byte multiple times.
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);

    // Contrarily, the peripheral does not need to be incremented - the SPI
    // data register doesn't move around as we write to it.
    dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL5);

    // We want to use 8 bit transfers
    dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_16BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);

    // We don't have any other DMA transfers going, but if we did we can use
    // priorities to try to ensure time-critical transfers are not interrupted
    // by others. In this case, it is alone.
    dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_HIGH);

    // Since we need to pull the register clock high after the transfer is
    // complete, enable transfer complete interrupts.
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL5);
    dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL5);
    dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL5);

    dma_enable_circular_mode(DMA1, DMA_CHANNEL5);

    // We also need to enable the relevant interrupt in the interrupt
    // controller, and assign it a priority.

    nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);
    nvic_set_priority(NVIC_DMA1_CHANNEL5_IRQ, 1);

    // Note - manipulating the memory address/size of the DMA controller cannot
    // be done while the channel is enabled. Ensure any previous transfer has
    // completed and the channel is disabled before you start another transfer.
    // Tell the DMA controller to start reading memory data from this address
    dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)data);

    // Configure the number of bytes to transfer
    dma_set_number_of_data(DMA1, DMA_CHANNEL5, BUFFER_SIZE);
    // Enable the DMA channel.
    dma_enable_channel(DMA1, DMA_CHANNEL5);


}

static void usart_setup() {
    // For the peripheral to work, we need to enable it's clock
    rcc_periph_clock_enable(RCC_USART1);
    // From the datasheet for the STM32F0 series of chips (Page 30, Table 11)
    // we know that the USART1 peripheral has it's TX line connected as
    // alternate function 1 on port A pin 9.
    // In order to use this pin for the alternate function, we need to set the
    // mode to GPIO_MODE_AF (alternate function). We also do not need a pullup
    // or pulldown resistor on this pin, since the peripheral will handle
    // keeping the line high when nothing is being transmitted.
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    // Now that we have put the pin into alternate function mode, we need to
    // select which alternate function to use. PA9 can be used for several
    // alternate functions - Timer 15, USART1 TX, Timer 1, and on some devices
    // I2C. Here, we want alternate function 1 (USART1_TX)
    // gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
    // Now that the pins are configured, we can configure the USART itself.
    // First, let's set the baud rate at 115200
    usart_set_baudrate(USART1, 115200);
    // Each datum is 8 bits
    usart_set_databits(USART1, 8);
    // No parity bit
    usart_set_parity(USART1, USART_PARITY_NONE);
    // One stop bit
    usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
    // For a debug console, we only need unidirectional transmit
    usart_set_mode(USART1, USART_MODE_TX);
    // No flow control
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    // Enable the peripheral
    usart_enable(USART1);

    // Optional extra - disable buffering on stdout.
    // Buffering doesn't save us any syscall overhead on embedded, and
    // can be the source of what seem like bugs.
    setbuf(stdout, NULL);
}

void uart_puts(char *string) {
    while (*string) {
        usart_send_blocking(USART1, *string);
        string++;
    }
}

void uart_putln(char *string) {
    uart_puts(string);
    uart_puts("\r\n");
}


static void pwm_setup(){

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM3_CH1);
    
    /* Clock division and mode */
    TIM3_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;

    /* Period */
    TIM3_ARR = 255;

    /* Prescaler */
    TIM3_PSC = 0;
    TIM3_EGR = TIM_EGR_UG;

    /* Output compare 1 mode and preload */
	TIM3_CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;

    /* Polarity and state */
	TIM3_CCER |= TIM_CCER_CC1P | TIM_CCER_CC1E;

    /* Capture compare value */
	TIM3_CCR1 = 0;

    /* ARR reload enable */
	TIM3_CR1 |= TIM_CR1_ARPE;

    /* Counter enable */
	TIM3_CR1 |= TIM_CR1_CEN;

}

void hxcmod_setup(){

    memset(data, 0, BUFFER_SIZE * sizeof(msample));

    hxcmod_init(&mctx);
    hxcmod_setcfg(&mctx, 44100, 0, 0);
    hxcmod_load(&mctx, (void *)MOD_sober_wednesday10, sizeof(MOD_sober_wednesday10));
    hxcmod_fillbuffer(&mctx, data, BUFFER_SIZE, &mod_state);

}

static void systick_setup() {
    // Set the systick clock source to our main clock
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    // Clear the Current Value Register so that we start at 0
    STK_CVR = 0;
    // In order to trigger an interrupt every millisecond, we can set the reload
    // value to be the speed of the processor / 1000 - 1
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    // Enable interrupts from the system tick clock
    systick_interrupt_enable();
    // Enable the system tick counter
    systick_counter_enable();
}

// Get the current value of the millis counter
uint64_t millis() {
    return _millis;
}
// This is our interrupt handler for the systick reload interrupt.
// The full list of interrupt services routines that can be implemented is
// listed in libopencm3/include/libopencm3/stm32/f1/nvic.h
void sys_tick_handler(void) {
    // Increment our monotonic clock
    _millis++;
}

// Delay a given number of milliseconds in a blocking manner
void delay(uint64_t duration) {
    const uint64_t until = millis() + duration;
    while (millis() < until);
}


int main() {
    // First, let's ensure that our clock is running off the high-speed
    // internal oscillator (HSI) at 48MHz
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Since our LED is on GPIO bank A, we need to enable
    // the peripheral clock to this GPIO bank in order to use it.
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_TIM1);

    // Our test LED is connected to Port A pin 11, so let's set it as output
    //gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_PUPD_NONE, GPIO11);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO11);
    
    idx = 0;
    buf = 0;


    systick_setup();
    hxcmod_setup();
    pwm_setup();
    timer_setup();
    usart_setup();
    dma_setup();



    printf("%s\n", mctx.song.title);


    // Now, let's forever toggle this LED back and forth
    while (true) {
    }

    return 0;
}
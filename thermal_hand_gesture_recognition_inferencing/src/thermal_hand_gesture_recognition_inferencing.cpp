#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_MLX90640.h>
#include <Thermal_Image_Inferencing.h>
#include <SPI.h>

#include "Particle.h"

#define TFT_DC D8
#define TFT_CS D9
#define FRAME_WIDTH 24
#define FRAME_HEIGHT 32
#define IMG_WIDTH 24
#define IMG_HEIGHT 32
#define PADDED_IMG_WIDTH 32
#define PADDED_IMG_HEIGHT 32
#define RESIZED_IMG_WIDTH 180
#define RESIZED_IMG_HEIGHT 240
#define ALIGN_PTR(p, a) \
    ((p & (a - 1)) ? (((uintptr_t)p + a) & ~(uintptr_t)(a - 1)) : p)

SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

void loadPalette();
bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, int src_len);
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

float frame[FRAME_WIDTH * FRAME_HEIGHT];
uint16_t img[IMG_WIDTH * IMG_HEIGHT];
uint16_t padded_img[PADDED_IMG_WIDTH * PADDED_IMG_HEIGHT] = {0};
uint16_t resized_img[RESIZED_IMG_WIDTH * RESIZED_IMG_HEIGHT];
uint16_t colorPal[256];
// uint8_t *ei_camera_capture_out = NULL;
uint8_t ei_camera_capture_out[EI_CLASSIFIER_INPUT_WIDTH *
                              EI_CLASSIFIER_INPUT_HEIGHT * 3];

Adafruit_ILI9341 tft(TFT_CS, TFT_DC);
Adafruit_MLX90640 mlx;

const char *labels[] = {"Unknown", "Rock", "Scissor", "Paper"};

void setup() {
    Serial.begin(115200);

    while (!Serial) {
        delay(100);
    }

    Serial.println("Inferencing");

    if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
        Serial.println("MLX90640 not found!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("Found Adafruit MLX90640");

    mlx.setMode(MLX90640_CHESS);
    mlx.setResolution(MLX90640_ADC_16BIT);
    mlx.setRefreshRate(MLX90640_8_HZ);
    Wire.setClock(400000);  // 400 KHz

    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(187, 10);
    tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    tft.print("Prediction");

    loadPalette();

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tImage resolution: %dx%d\n", EI_CLASSIFIER_INPUT_WIDTH,
              EI_CLASSIFIER_INPUT_HEIGHT);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tNo. of classes: %d\n",
              sizeof(ei_classifier_inferencing_categories) /
                  sizeof(ei_classifier_inferencing_categories[0]));
}

uint8_t prev_max_prob_index = 0;
unsigned long int prev_pub_ms = 0;

void loop() {
    if (mlx.getFrame(frame) != 0) {
        Serial.println("Failed");
        return;
    }

    auto minmax =
        std::minmax_element(frame, frame + FRAME_WIDTH * FRAME_HEIGHT);
    int min_temp = int(*minmax.first);
    int max_temp = int(*minmax.second);

    Serial.printf("Min temp = %d C, Max Temp: %d C\n", min_temp, max_temp);

    for (int pixel = 0, h = 0; h < FRAME_HEIGHT; h++) {
        for (int w = 0; w < FRAME_WIDTH; w++) {
            float t = frame[FRAME_HEIGHT * w + h];

            float scaledPix = constrain(
                (t - (min_temp + 3)) / (32.0 - (min_temp + 3)) * 255.9, 0.0,
                255.0);

            img[pixel++] = colorPal[(uint16_t)scaledPix];
        }
    }

    for (int i = 0, j = 0, h = 0; h < FRAME_HEIGHT; h++) {
        i += 4;
        for (int w = 0; w < FRAME_WIDTH; w++) {
            padded_img[i++] = img[j++];
        }
        i += 4;
    }

    bool converted =
        RBG565ToRGB888((uint8_t *)padded_img, ei_camera_capture_out,
                       PADDED_IMG_HEIGHT * PADDED_IMG_WIDTH * 2);

    if (!converted) {
        ei_printf("ERR: Conversion failed\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length =
        EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    // run the impulse: DSP, neural network and the Anomaly algorithm
    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR ei_error = run_classifier(&signal, &result);
    if (ei_error != EI_IMPULSE_OK) {
        ei_printf("Failed to run impulse (%d)\n", ei_error);
        return;
    }

    // print the predictions
    ei_printf(
        "Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): "
        "\n",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf("Predictions:\r\n");

    uint8_t max_prob_index;
    float max_prob_value;

    for (uint8_t index = 0; index < EI_CLASSIFIER_LABEL_COUNT; index++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[index]);
        ei_printf("%.5f\r\n", result.classification[index].value);

        if (result.classification[index].value > max_prob_value) {
            max_prob_value = result.classification[index].value;
            max_prob_index = index;
        }
    }

    if ((prev_max_prob_index != max_prob_index) &&
        (millis() - prev_pub_ms) > 1000) {
        Particle.publish("gestureEvent",
                         ei_classifier_inferencing_categories[max_prob_index],
                         PRIVATE);
        prev_pub_ms = millis();
    }

    prev_max_prob_index = max_prob_index;

    tft.setCursor(187, 30);
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.printf("%-7s", ei_classifier_inferencing_categories[max_prob_index]);
    tft.setCursor(187, 50);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.printf("%.2f%%", result.classification[max_prob_index].value * 100.0f);

    for (int y = 0; y < RESIZED_IMG_HEIGHT; y++) {
        for (int x = 0; x < RESIZED_IMG_WIDTH; x++) {
            int nearestX = (x * IMG_WIDTH) / RESIZED_IMG_WIDTH;
            int nearestY = (y * IMG_HEIGHT) / RESIZED_IMG_HEIGHT;
            resized_img[(y * RESIZED_IMG_WIDTH) + x] =
                img[(nearestY * IMG_WIDTH) + nearestX];
        }
    }

    tft.startWrite();
    tft.setAddrWindow(0, 0, RESIZED_IMG_WIDTH, RESIZED_IMG_HEIGHT);
    for (int i = 0; i < RESIZED_IMG_WIDTH * RESIZED_IMG_HEIGHT; i++) {
        tft.SPI_WRITE16(resized_img[i]);
    }
    tft.endWrite();
}

int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (ei_camera_capture_out[pixel_ix] << 16) +
                              (ei_camera_capture_out[pixel_ix + 1] << 8) +
                              ei_camera_capture_out[pixel_ix + 2];

        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }

    return 0;
}

bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, int src_len) {
    uint8_t hb, lb;
    int pix_count = src_len / 2;

    for (int i = 0; i < pix_count; i++) {
        hb = *src_buf++;
        lb = *src_buf++;

        *dst_buf++ = hb & 0xF8;
        *dst_buf++ = (hb & 0x07) << 5 | (lb & 0xE0) >> 3;
        *dst_buf++ = (lb & 0x1F) << 3;
    }

    return true;
}

void loadPalette() {
    float fleX, fleK;
    for (uint16_t x = 0; x < 256; ++x) {
        fleX = (float)x / 255.0;
        fleK = 63487.0 * (1.02 - (fleX - 0.72) * (fleX - 0.72) * 1.96);
        fleK = (fleK > 63487.0) || (fleX > 0.75) ? 63487.0
                                                 : fleK;  // Truncate red curve
        colorPal[x] = (uint16_t)fleK & 0xF800;  // Top 5 bits define red

        fleK = fleX * fleX * 2015.0;
        colorPal[x] += (uint16_t)fleK & 0x07E0;  // Middle 6 bits define green

        fleK = 30.9 * (14.0 * (fleX * fleX * fleX) - 20.0 * (fleX * fleX) +
                       7.0 * fleX);
        fleK = fleK < 0.0 ? 0.0 : fleK;          // Truncate blue curve
        colorPal[x] += (uint16_t)fleK & 0x001F;  // Bottom 5 bits define blue
    }
}

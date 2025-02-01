#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_MLX90640.h>
#include <JPEGENC.h>
#include <SDFat.h>
#include <SPI.h>

#include "Particle.h"

#define SD_CS S4
#define TFT_DC D8
#define TFT_CS D9
#define BTN_WHITE D6
#define BTN_RED D7

#define FRAME_WIDTH 24
#define FRAME_HEIGHT 32
#define IMG_WIDTH 24
#define IMG_HEIGHT 32
#define PADDED_IMG_WIDTH 32
#define PADDED_IMG_HEIGHT 32
#define RESIZED_IMG_WIDTH 180
#define RESIZED_IMG_HEIGHT 240

SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

void loadPalette();

float frame[FRAME_WIDTH * FRAME_HEIGHT];
uint16_t img[IMG_WIDTH * IMG_HEIGHT];
uint16_t padded_img[PADDED_IMG_WIDTH * PADDED_IMG_HEIGHT] = {0};
uint16_t resized_img[RESIZED_IMG_WIDTH * RESIZED_IMG_HEIGHT];
uint16_t colorPal[256];

Adafruit_ILI9341 tft(TFT_CS, TFT_DC);
Adafruit_MLX90640 mlx;

const char *labels[] = {"Unknown", "Rock", "Scissor", "Paper"};

uint8_t label_index = 0;
volatile bool change_label = false;
volatile bool save_image = false;
JPEGENC jpg;
SdFat SD;
bool sd_initialized = false;
static File myfile;

void *myOpen(const char *filename) {
    myfile = SD.open(filename, FILE_WRITE);
    return (void *)&myfile;
}

void myClose(JPEGE_FILE *p) {
    File *f = (File *)p->fHandle;
    if (f) f->close();
}

int32_t myRead(JPEGE_FILE *p, uint8_t *buffer, int32_t length) {
    File *f = (File *)p->fHandle;
    return f->read(buffer, length);
}

int32_t myWrite(JPEGE_FILE *p, uint8_t *buffer, int32_t length) {
    File *f = (File *)p->fHandle;
    return f->write(buffer, length);
}

int32_t mySeek(JPEGE_FILE *p, int32_t position) {
    File *f = (File *)p->fHandle;
    return f->seek(position);
}

void change_label_cb() { change_label = true; }

void save_image_cb() { save_image = true; }

void setup() {
    Serial.begin(115200);

    while (!Serial) {
        delay(100);
    }

    Serial.println("Data Collection");

    pinMode(BTN_RED, INPUT);
    pinMode(BTN_WHITE, INPUT);

    attachInterrupt(digitalPinToInterrupt(BTN_RED), change_label_cb, RISING);
    attachInterrupt(digitalPinToInterrupt(BTN_WHITE), save_image_cb, RISING);

    Serial.println("Initializing SD card...");

    if (!SD.begin(SD_CS, SD_SCK_MHZ(10))) {
        Serial.println("SD card initialization failed!");
    } else {
        sd_initialized = true;
        Serial.println("initialization done.");
    }

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
    tft.setCursor(187, 80);
    tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    tft.print("Label");
    tft.setCursor(187, 100);
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.printf("%-7s", labels[label_index]);

    loadPalette();
}

int idx = 0;
unsigned int count = 0;

void loop() {
    if (mlx.getFrame(frame) != 0) {
        Serial.println("Failed");
        return;
    }

    auto minmax =
        std::minmax_element(frame, frame + FRAME_WIDTH * FRAME_HEIGHT);
    int min_temp = int(*minmax.first);
    int max_temp = int(*minmax.second);

    tft.setCursor(187, 10);
    tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    tft.print("Temperature");
    tft.setCursor(187, 30);
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.printf("Min:%d%cC  ", min_temp, (char)247);
    tft.setCursor(187, 50);
    tft.printf("Max:%d%cC  ", max_temp, (char)247);

    for (int pixel = 0, h = 0; h < FRAME_HEIGHT; h++) {
        for (int w = 0; w < FRAME_WIDTH; w++) {
            int i = FRAME_HEIGHT * w + h;
            float t = frame[i];
            float scaledPix = constrain(
                (t - (min_temp + 3)) / (32.0 - (min_temp + 3)) * 255.9, 0.0,
                255.0);

            img[pixel++] = colorPal[(uint16_t)scaledPix];
        }
    }

    unsigned long start_ms = millis();

    for (int y = 0; y < RESIZED_IMG_HEIGHT; y++) {
        for (int x = 0; x < RESIZED_IMG_WIDTH; x++) {
            int nearestX = (x * IMG_WIDTH) / RESIZED_IMG_WIDTH;
            int nearestY = (y * IMG_HEIGHT) / RESIZED_IMG_HEIGHT;
            resized_img[(y * RESIZED_IMG_WIDTH) + x] =
                img[(nearestY * IMG_WIDTH) + nearestX];
        }
    }

    Serial.printf("Interpolation = %ld ms\n", millis() - start_ms);

    tft.startWrite();
    tft.setAddrWindow(0, 0, RESIZED_IMG_WIDTH, RESIZED_IMG_HEIGHT);
    for (int i = 0; i < RESIZED_IMG_WIDTH * RESIZED_IMG_HEIGHT; i++) {
        tft.SPI_WRITE16(resized_img[i]);
    }
    tft.endWrite();

    if (change_label) {
        change_label = false;
        label_index = (label_index + 1) % 4;
        tft.setCursor(187, 100);
        tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
        tft.printf("%-7s", labels[label_index]);
    }

    if (sd_initialized && save_image) {
        save_image = false;
        tft.setCursor(187, 130);
        tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
        tft.printf("%-9s", "Saving...");
        delay(500);

        // padding the image to 32 x 32 for the training
        for (int i = 0, j = 0, h = 0; h < FRAME_HEIGHT; h++) {
            i += 4;
            for (int w = 0; w < FRAME_WIDTH; w++) {
                padded_img[i++] = img[j++];
            }
            i += 4;
        }

        JPEGENCODE enc;
        char filename[20];
        sprintf(filename, "%s/%s.%03d.jpg", labels[label_index],
                labels[label_index], count++);

        int rc = jpg.open(filename, myOpen, myClose, myRead, myWrite, mySeek);
        if (rc == JPEGE_SUCCESS) {
            jpg.encodeBegin(&enc, PADDED_IMG_WIDTH, PADDED_IMG_HEIGHT,
                            JPEGE_PIXEL_RGB565, JPEGE_SUBSAMPLE_444,
                            JPEGE_Q_HIGH);
            jpg.addFrame(&enc, (uint8_t *)padded_img, PADDED_IMG_WIDTH * 2);
            jpg.close();
            tft.setCursor(187, 130);
            tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
            tft.printf("%-9s", "Saved!");
        } else {
            tft.setCursor(187, 130);
            tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
            tft.printf("%-9s", "Failed!");
        }
    }
}

void loadPalette() {
    float fleX, fleK;
    for (uint16_t x = 0; x < 256; ++x) {
        fleX = (float)x / 255.0;
        fleK = 63487.0 * (1.02 - (fleX - 0.72) * (fleX - 0.72) * 1.96);
        fleK = (fleK > 63487.0) || (fleX > 0.75) ? 63487.0 : fleK;
        colorPal[x] = (uint16_t)fleK & 0xF800;
        fleK = fleX * fleX * 2015.0;
        colorPal[x] += (uint16_t)fleK & 0x07E0;
        fleK = 30.9 * (14.0 * (fleX * fleX * fleX) - 20.0 * (fleX * fleX) +
                       7.0 * fleX);
        fleK = fleK < 0.0 ? 0.0 : fleK;
        colorPal[x] += (uint16_t)fleK & 0x001F;
    }
}

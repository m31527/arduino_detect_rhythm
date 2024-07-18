#include <SD.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <cmath>

// 定义引脚
#define MY_CS   33
#define MY_SCLK 25
#define MY_MISO 27
#define MY_MOSI 26
#define BUTTON1 35
#define BUTTON2 0

TFT_eSPI tft = TFT_eSPI();
const uint16_t samples = 256;
const double samplingFrequency = 1000;

File wavFile;
File root;

// WAV file header格式
typedef struct {
    char chunkID[4];       // "RIFF"
    uint32_t chunkSize;
    char format[4];        // "WAVE"
    char subchunk1ID[4];   // "fmt "
    uint32_t subchunk1Size;
    uint16_t audioFormat;
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
    char subchunk2ID[4];   // "data"
    uint32_t subchunk2Size;
} WAVHeader;

bool read_wav_segment(const char* filename, int segment_start, int segment_size, float* segment, int* sample_rate);

void calculate_envelope(float* audio_data, int num_samples, float* envelope);
int detect_peaks(float* envelope, int num_samples, int* peaks, int distance);
void calculate_peak_intervals(int* peaks, int peak_count, int* intervals);
void calculate_statistics(int* intervals, int count, double* mean, double* stddev);

void setup() {
    pinMode(BUTTON1, INPUT_PULLUP);
    pinMode(BUTTON2, INPUT_PULLUP);

    Serial.begin(115200);
    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);

    SPI.begin(MY_SCLK, MY_MISO, MY_MOSI, MY_CS);

    if (!SD.begin(MY_CS)) {
        Serial.println("SD card initialization failed!");
        tft.setCursor(20, 60);
        tft.setTextColor(TFT_RED);
        tft.setTextSize(3);
        tft.println("SD Init Failed!");
        return;
    }
    Serial.println("SD card initialized.");
    tft.setCursor(20, 20);
    tft.setTextColor(TFT_GREEN);
    tft.setTextSize(1);
    tft.println("SD Init Success!");

    const char* filename = "/Noisy2.wav";
    int num_samples, sample_rate;
    int segment_size = 2000;  // 每次读取1000个样本
    float* segment = (float*)malloc(segment_size * sizeof(float));
    float* envelope = (float*)malloc(segment_size * sizeof(float));
    int* peaks = (int*)malloc(segment_size * sizeof(int));
    int* intervals = (int*)malloc((segment_size - 1) * sizeof(int));

    if (!segment || !envelope || !peaks || !intervals) {
        Serial.println("Memory allocation failed");
        if (segment) free(segment);
        if (envelope) free(envelope);
        if (peaks) free(peaks);
        if (intervals) free(intervals);
        return;
    }

    int segment_start = 0;
    int peak_count_total = 0;
    double mean_total = 0.0, stddev_total = 0.0;

    while (read_wav_segment(filename, segment_start, segment_size, segment, &sample_rate)) {
        segment_start += segment_size;

        calculate_envelope(segment, segment_size, envelope);
        int peak_count = detect_peaks(envelope, segment_size, peaks, 10);  // 调整距离参数

        // Debug: 打印检测到的峰值
        Serial.print("Detected peaks: ");
        for (int i = 0; i < peak_count; i++) {
            Serial.print(peaks[i]);
            Serial.print(" ");
        }
        Serial.println();

        int interval_count = peak_count - 1;
        if (interval_count > 0) {
            calculate_peak_intervals(peaks, peak_count, intervals);

            // Debug: 打印间隔
            Serial.print("Calculated intervals: ");
            for (int i = 0; i < interval_count; i++) {
                Serial.print(intervals[i]);
                Serial.print(" ");
            }
            Serial.println();

            double mean_interval, std_interval;
            calculate_statistics(intervals, interval_count, &mean_interval, &std_interval);

            mean_total += mean_interval * interval_count;
            stddev_total += std_interval * std_interval * interval_count;
            peak_count_total += interval_count;

            // 每段的Debug输出
            Serial.print("Segment start: ");
            Serial.print(segment_start);
            Serial.print(", Mean Interval: ");
            Serial.print(mean_interval);
            Serial.print(", Std Deviation: ");
            Serial.println(std_interval);
        } else {
            Serial.println("No intervals to calculate.");
        }

        if (segment_start >= segment_size * 3) {  // 只处理三段
            break;
        }
    }

    if (peak_count_total > 0) {
        mean_total /= peak_count_total;
        stddev_total = sqrt(stddev_total / peak_count_total);
    }
    double percentage = (stddev_total / mean_total) * 100.0;

    // 输出结果
    Serial.print("Mean Interval: ");
    Serial.println(mean_total);
    Serial.print("Standard Deviation: ");
    Serial.println(stddev_total);
    Serial.print("Percentage: ");
    Serial.println(percentage);

    tft.setCursor(20, 40);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.println("Results:");
    tft.print("Mean Interval: ");
    tft.println(mean_total);
    tft.print("Std Deviation: ");
    tft.println(stddev_total);
    tft.print("Percentage: ");
    tft.println(percentage);

    // 释放内存
    free(segment);
    free(envelope);
    free(peaks);
    free(intervals);
}

void loop() {
    // 此示例不需要在循环中重复操作
}

bool read_wav_segment(const char* filename, int segment_start, int segment_size, float* segment, int* sample_rate) {
    File file = SD.open(filename);
    if (!file) {
        Serial.print("Unable to open file ");
        Serial.println(filename);
        return false;
    }

    WAVHeader header;
    file.read((uint8_t*)&header, sizeof(WAVHeader));

    if (header.audioFormat != 1 || header.bitsPerSample != 16) {
        Serial.println("Unsupported WAV format");
        file.close();
        return false;
    }

    *sample_rate = header.sampleRate;
    file.seek(44 + segment_start * 2);  // 跳过头部并移动到段开始

    int16_t* buffer = (int16_t*)malloc(segment_size * sizeof(int16_t));
    if (!buffer) {
        Serial.println("Memory allocation failed for buffer");
        file.close();
        return false;
    }

    file.read((uint8_t*)buffer, segment_size * sizeof(int16_t));
    file.close();

    for (int i = 0; i < segment_size; i++) {
        segment[i] = buffer[i] / 32768.0f;  // 归一化到[-1.0, 1.0]
    }

    free(buffer);
    return true;
}

void calculate_envelope(float* audio_data, int num_samples, float* envelope) {
    for (int i = 0; i < num_samples; i++) {
        envelope[i] = fabs(audio_data[i]);
    }
}

int detect_peaks(float* envelope, int num_samples, int* peaks, int distance) {
    int peak_count = 0;
    for (int i = distance; i < num_samples - distance; i++) {
        int is_peak = 1;
        for (int j = 1; j <= distance; j++) {
            if (envelope[i] < envelope[i - j] || envelope[i] <= envelope[i + j]) {
                is_peak = 0;
                break;
            }
        }
        if (is_peak) {
            peaks[peak_count++] = i;
        }
    }
    return peak_count;
}

void calculate_peak_intervals(int* peaks, int peak_count, int* intervals) {
    for (int i = 1; i < peak_count; i++) {
        intervals[i - 1] = peaks[i] - peaks[i - 1];
    }
}

void calculate_statistics(int* intervals, int count, double* mean, double* stddev) {
    if (count == 0) {
        *mean = 0.0;
        *stddev = 0.0;
        return;
    }

    double sum = 0.0;
    for (int i = 0; i < count; i++) {
        sum += intervals[i];
    }
    *mean = sum / count;

    double variance = 0.0;
    for (int i = 0; i < count; i++) {
        variance += (intervals[i] - *mean) * (intervals[i] - *mean);
    }
    *stddev = sqrt(variance / count);
}

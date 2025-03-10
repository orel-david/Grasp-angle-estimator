#include "esp_camera.h"
#include <WiFi.h>
#include "utils.h"
#include "segment.h"
#include "fb_gfx.h"
#include <cstring>

// Replace with your network credentials
const char* ssid = "OrelDavid";
const char* password = "12345678";

// Camera pin configuration (for AI Thinker module)
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27

#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

WiFiServer server(80);

void startCamera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Set frame size and buffer
  config.frame_size = FRAMESIZE_QVGA; // 320x240 resolution
  config.jpeg_quality = 5;         
  config.fb_count = 1;
  config.pixel_format = PIXFORMAT_GRAYSCALE;



  // Initialize the camera
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (1);
  }
  sensor_t *sensor = esp_camera_sensor_get();

    // Check if sensor object is valid
  if (!sensor) {
    Serial.println("Failed to get sensor");
    return;
  }

    // Change exposure settings
  sensor->set_exposure_ctrl(sensor, 1); // Enable automatic exposure control
  sensor->set_aec_value(sensor, 500); // Manually set exposure value (0-1200)
}

void startWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("Camera Stream Ready! Visit: http://");
  Serial.println(WiFi.localIP());

  server.begin();
}

void handleClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  // Wait for the client to send a request
  while (!client.available()) {
    delay(1);
  }

  // Send HTTP headers for MJPEG streaming
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println();

  while (client.connected()) {

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        continue;
    }

    uint8_t *jpg_buf = nullptr;
    size_t jpg_buf_len = 0;
    int width = fb->width;
    int height = fb->height;

    uint8_t* temp = (uint8_t*) calloc(2*width * height, sizeof(uint8_t));


    // Apply Gaussian blur
    unsigned long start_time = millis();
    for(int y = 0; y < height; y++)
    {
      for(int x = 0; x < width; x++)
      {
        temp[y * (2*width) + x] = (fb->buf)[y * width + x];  // Next two columns

      }
    }
    std::vector<Hull> hulls = segment(fb->buf, width, height);
    unsigned long end_time = millis();
    uint8_t* output = imageFromHulls(hulls);
    if(output == nullptr)
    {
      continue;
    }
    for(int y = 0; y < height; y++)
    {
      for(int x = 0; x < width; x++)
      {
        temp[y * (2*width) + x + width] = (output)[y * width + x];  // Next two columns

      }
    }
    Serial.println("Execution Time: " + String(end_time - start_time) + " ms");

    // Convert to JPEG if grayscale TODO REMOVE AFTER DEBUG
    if (fb->format == PIXFORMAT_GRAYSCALE) {
        if (!fmt2jpg(temp, 2*fb->len, 2*width, height, PIXFORMAT_GRAYSCALE, 80, &jpg_buf, &jpg_buf_len)) {
            Serial.println("JPEG encoding failed");
            esp_camera_fb_return(fb);
            continue;
        }
    } else {
        // Use the original JPEG buffer
        jpg_buf = fb->buf;
        jpg_buf_len = fb->len;
    }

    // Send the frame in MJPEG format
    client.println("--frame");
    client.println("Content-Type: image/jpeg");
    client.println("Content-Length: " + String(jpg_buf_len));
    client.println();
    client.write(jpg_buf, jpg_buf_len);
    client.println();

    // Free the allocated buffer if grayscale conversion was done
    if (fb->format == PIXFORMAT_GRAYSCALE) {
        free(jpg_buf);
    }
    free(output);
    free(temp);
    esp_camera_fb_return(fb);
  }
}

void setup() {
  Serial.begin(115200);
  startCamera();
  startWiFi();
  Serial.println("hi");
  Serial.print("norm sanity check: ");
  Serial.println(norm(1, 2)); 
}

void loop() {
  handleClient();
}

#include "esp_camera.h"
#include <Arduino.h>
#include "esp_log.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <mbedtls/base64.h>

static const char* TAG = "camera";

// Base64 encoding helper
String base64_encode(const uint8_t* data, size_t len) {
    size_t encoded_len = 0;
    mbedtls_base64_encode(nullptr, 0, &encoded_len, data, len);
    
    unsigned char* encoded = (unsigned char*)malloc(encoded_len + 1);
    if (!encoded) return "";
    
    mbedtls_base64_encode(encoded, encoded_len + 1, &encoded_len, data, len);
    String result = String((const char*)encoded);
    free(encoded);
    return result;
}

// -------------------- PIN MAP --------------------
// 4D Systems ESP32-S3 Gen4 (adjust if needed)
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD    4   // SCCB Data
#define CAM_PIN_SIOC    5   // SCCB Clock
#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2      8
#define CAM_PIN_D1      9
#define CAM_PIN_D0      11
#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK    13


// ------------------ CAMERA CONFIG ------------------
camera_config_t camera_config_template = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 24000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA, // QQVGA-UXGA, JPEG mode recommended, ?FRAMESIZE_VGA?
    .jpeg_quality = 25,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY
};

// ----------------- RUNTIME SETTINGS -----------------
int current_capture_quality = 4;
const int capture_qualities[] = {0, 2, 4, 6, 8, 12, 18, 25, 35, 45, 63};
const int capture_qualities_len = sizeof(capture_qualities) / sizeof(capture_qualities[0]);

// Stream settings
framesize_t stream_framesizes[] = {FRAMESIZE_QVGA, FRAMESIZE_VGA, FRAMESIZE_SVGA, FRAMESIZE_XGA, FRAMESIZE_UXGA};
const char* stream_framesize_names[] = {"QVGA", "VGA", "SVGA", "XGA", "UXGA"};
const int stream_framesize_len = sizeof(stream_framesizes) / sizeof(stream_framesizes[0]);
int stream_framesize_index = 1; // default VGA
int current_stream_fb_count = 1;


// ------------------- WIFI -------------------
const char* ssid = "FRITZ!Box 7590 KP";
const char* password = "!1234567890!1234";

// ------------------- SERVER -------------------
WebServer server(80);

// ------------------- SMTP SETTINGS -----------------
const char* smtp_host = "smtp.gmail.com";
const int smtp_port = 465;
const char* smtp_user = "14peterstadler@gmail.com";
const char* smtp_pass = "rrbt glgn neuk izwk";
const char* smtp_from = "14peterstadler@gmail.com";
const char* smtp_to = "14peterstadler@gmail.com";

// ----------------- CAMERA INIT -----------------
esp_err_t init_camera(framesize_t frame_size, int jpeg_quality, int fb_count) {
    camera_config_t cfg = camera_config_template;  // copy template
    cfg.frame_size = frame_size;
    cfg.jpeg_quality = jpeg_quality;
    cfg.fb_count = fb_count;

    if(CAM_PIN_PWDN != -1){
        pinMode(CAM_PIN_PWDN, OUTPUT);
        digitalWrite(CAM_PIN_PWDN, LOW);
    }

    esp_camera_deinit();
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed: 0x%x", err);
        return err;
    }

    Serial.println("Camera initialized successfully!");
    return ESP_OK;
}

// ----------------- STREAM -----------------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

void handle_stream() {
    if(init_camera(stream_framesizes[stream_framesize_index], 25, current_stream_fb_count) != ESP_OK) { // VGA + medium quality
        server.send(500, "text/plain", "Stream camera init failed");
        return;
    }
    WiFiClient client = server.client();
    client.setNoDelay(true); // <--- CRITICAL: Removes TCP buffering lag

    // Send the initial MJPEG header
    server.sendContent("HTTP/1.1 200 OK\r\n"
                       "Content-Type: " + String(_STREAM_CONTENT_TYPE) + "\r\n"
                       "Content-Length: -1\r\n" // Chunked/Streamed
                       "Access-Control-Allow-Origin: *\r\n"
                       "\r\n");

 while (client.connected()) {
        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) continue;

        char part_buf[64];
        size_t hlen = snprintf(part_buf, 64, _STREAM_PART, fb->len);

        if (!client.write(_STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY)) ||
            !client.write(part_buf, hlen) ||
            !client.write(fb->buf, fb->len)) {
            esp_camera_fb_return(fb);
            break;
        }

        esp_camera_fb_return(fb);

        unsigned long start = millis();
        while(millis() - start < 200) { // 5 FPS
            server.handleClient();
            yield();
        }
    }
}

// ----------------- SINGLE CAPTURE -----------------
void handle_jpg() {
    if(init_camera(FRAMESIZE_UXGA, current_capture_quality, 1) != ESP_OK) { // UXGA + current dynamic quality
        server.send(500, "text/plain", "Capture camera init failed");
        return;
    }

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        server.send(500, "text/plain", "Camera capture failed");
        return;
    }

    server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.setContentLength(fb->len);
    server.send(200, "image/jpeg", "");

    WiFiClient client = server.client();
    client.write(fb->buf, fb->len);

    esp_camera_fb_return(fb);
}

// --------- SIMPLE EMAIL SEND ---------
bool send_email_with_fb(const char* to_addr, camera_fb_t * fb) {
    if (!fb) return false;
    
    WiFiClientSecure client;
    client.setInsecure(); // Skip certificate verification for simplicity
    
    if (!client.connect(smtp_host, smtp_port)) {
        Serial.println("SMTP connection failed");
        return false;
    }
    
    // Read SMTP response
    delay(100);
    while (client.available()) client.readStringUntil('\n');
    
    // Send EHLO
    client.println("EHLO ESP32");
    delay(100);
    while (client.available()) client.readStringUntil('\n');
    
    // Login
    client.println("AUTH LOGIN");
    delay(100);
    while (client.available()) client.readStringUntil('\n');
    
    client.println(base64_encode((uint8_t*)smtp_user, strlen(smtp_user)));
    delay(100);
    while (client.available()) client.readStringUntil('\n');
    
    client.println(base64_encode((uint8_t*)smtp_pass, strlen(smtp_pass)));
    delay(100);
    while (client.available()) client.readStringUntil('\n');
    
    // Send mail
    client.print("MAIL FROM:<");
    client.print(smtp_user);
    client.println(">");
    delay(100);
    while (client.available()) client.readStringUntil('\n');
    
    client.print("RCPT TO:<");
    client.print(to_addr);
    client.println(">");
    delay(100);
    while (client.available()) client.readStringUntil('\n');
    
    client.println("DATA");
    delay(100);
    while (client.available()) client.readStringUntil('\n');
    
    // Email headers
    String boundary = "boundary123456789";
    client.print("From: ");
    client.println(smtp_user);
    client.print("To: ");
    client.println(to_addr);
    client.println("Subject: ESP32 Photo");
    client.println("MIME-Version: 1.0");
    client.print("Content-Type: multipart/mixed; boundary=\"");
    client.print(boundary);
    client.println("\"");
    client.println();
    
    // Text part
    client.print("--");
    client.println(boundary);
    client.println("Content-Type: text/plain");
    client.println();
    client.println("Photo from ESP32 Camera");
    client.println();
    
    // Image attachment part
    client.print("--");
    client.println(boundary);
    client.println("Content-Type: image/jpeg; name=\"photo.jpg\"");
    client.println("Content-Transfer-Encoding: base64");
    client.println("Content-Disposition: attachment; filename=\"photo.jpg\"");
    client.println();
    
    // Base64 encode image with proper line wrapping (76 chars per line)
    String img_b64 = base64_encode(fb->buf, fb->len);
    const int LINE_LENGTH = 76;
    for (size_t i = 0; i < img_b64.length(); i += LINE_LENGTH) {
        int end = (i + LINE_LENGTH < img_b64.length()) ? i + LINE_LENGTH : img_b64.length();
        client.println(img_b64.substring(i, end));
    }
    
    // End boundary
    client.println();
    client.print("--");
    client.print(boundary);
    client.println("--");
    client.println(".");
    delay(100);
    while (client.available()) client.readStringUntil('\n');
    
    client.println("QUIT");
    client.stop();
    
    return true;
}

String base64_encode(uint8_t* data, size_t len) {
    static const char* base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    String encoded = "";
    
    for (size_t i = 0; i < len; i += 3) {
        uint8_t b1 = data[i];
        uint8_t b2 = (i + 1 < len) ? data[i + 1] : 0;
        uint8_t b3 = (i + 2 < len) ? data[i + 2] : 0;
        
        int n = (b1 << 16) | (b2 << 8) | b3;
        encoded += base64_chars[(n >> 18) & 0x3F];
        encoded += base64_chars[(n >> 12) & 0x3F];
        encoded += (i + 1 < len) ? base64_chars[(n >> 6) & 0x3F] : '=';
        encoded += (i + 2 < len) ? base64_chars[n & 0x3F] : '=';
    }
    
    return encoded;
}

// ----------------- UI / CONTROL HANDLERS -----------------
void handle_capture_page() {
    String html = "<h1>Capture</h1>";
    html += "<p>Current JPEG quality: <span id=\"q\">" + String(current_capture_quality) + "</span></p>";
    html += "<button onclick=\"fetch('/quality?dir=down').then(()=>location.reload())\">&#9664;</button>";
    html += "<button onclick=\"fetch('/quality?dir=up').then(()=>location.reload())\">&#9654;</button>";
    html += "<p>";
    html += "<button id=\"emailBtn\" onclick=\"sendEmail()\">Send via Email</button>";
    html += "<span id=\"status\" style=\"margin-left:10px; font-weight:bold;\"></span>";
    html += "</p>";
    html += "<script>";
    html += "function sendEmail() {";
    html += "  const btn = document.getElementById('emailBtn');";
    html += "  const status = document.getElementById('status');";
    html += "  btn.disabled = true;";
    html += "  status.innerText = '...sending...';";
    html += "  status.style.color = 'orange';";
    html += "  fetch('/send_email').then(r => {";
    html += "    if(r.ok) {";
    html += "      status.innerText = '✅ Email sent';";
    html += "      status.style.color = 'green';";
    html += "    } else {";
    html += "      return r.text().then(t => {";
    html += "        status.innerText = '❌ ' + t;";
    html += "        status.style.color = 'red';";
    html += "      });";
    html += "    }";
    html += "  }).catch(e => {";
    html += "    status.innerText = '❌ Error: ' + e.message;";
    html += "    status.style.color = 'red';";
    html += "  }).finally(() => {";
    html += "    btn.disabled = false;";
    html += "  });";
    html += "}";
    html += "</script>";
    html += "<p><a href='/'>Back</a></p>";
    html += "<p><img id=\"photo\" src=\"/capture.jpg?ts=" + String(millis()) + "\" style=\"max-width:100%\"></p>";
    server.send(200, "text/html", html);
}

void handle_send_email() {
    if(init_camera(FRAMESIZE_UXGA, current_capture_quality, 1) != ESP_OK) {
        server.send(500, "text/plain", "Camera init failed");
        return;
    }
    
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        server.send(500, "text/plain", "Capture failed");
        return;
    }
    
    bool ok = send_email_with_fb(smtp_to, fb);
    esp_camera_fb_return(fb);
    
    server.send(ok ? 200 : 500, "text/plain", ok ? "✅ Email sent" : "❌ Email send failed");
}

void handle_quality_change() {
    // Change current_capture_quality using capture_qualities array
    String dir = server.arg("dir");
    int idx = 0;
    for (int i = 0; i < capture_qualities_len; ++i) if (capture_qualities[i] == current_capture_quality) idx = i;

    if (dir == "up") {
        if (idx < capture_qualities_len - 1) idx++;
    } else if (dir == "down") {
        if (idx > 0) idx--;
    } else if (server.hasArg("value")) {
        int v = server.arg("value").toInt();
        // find nearest
        for (int i = 0; i < capture_qualities_len; ++i) if (capture_qualities[i] == v) idx = i;
    }
    current_capture_quality = capture_qualities[idx];
    server.send(200, "application/json", String("{\"quality\":") + String(current_capture_quality) + String("}"));
}

void handle_stream_page() {
    String html = "<h1>Live Stream</h1>";
    html += "<p>Frame size: <span id=\"fs\">" + String(stream_framesize_names[stream_framesize_index]) + "</span> | fb_count: <span id=\"fb\">" + String(current_stream_fb_count) + "</span></p>";
    html += "<button onclick=\"fetch('/stream_ctrl?action=framesize&dir=down').then(()=>location.reload())\">Frame &#9664;</button>";
    html += "<button onclick=\"fetch('/stream_ctrl?action=framesize&dir=up').then(()=>location.reload())\">Frame &#9654;</button>";
    html += "<button onclick=\"fetch('/stream_ctrl?action=fb_count&value=1').then(()=>location.reload())\">FB:1</button>";
    html += "<button onclick=\"fetch('/stream_ctrl?action=fb_count&value=2').then(()=>location.reload())\">FB:2</button>";
    html += "<p><a href='/'>Back</a></p>";
    html += "<p><img src=\"/stream.mjpg\" style=\"max-width:100%\"></p>";
    server.send(200, "text/html", html);
}

void handle_stream_ctrl() {
    String action = server.arg("action");
    if (action == "framesize") {
        String dir = server.arg("dir");
        if (dir == "up") {
            if (stream_framesize_index < stream_framesize_len - 1) stream_framesize_index++;
        } else if (dir == "down") {
            if (stream_framesize_index > 0) stream_framesize_index--;
        }
        server.send(200, "application/json", String("{\"framesize\":\"") + String(stream_framesize_names[stream_framesize_index]) + String("\"}"));
        return;
    } else if (action == "fb_count") {
        int v = server.arg("value").toInt();
        if (v == 1 || v == 2) current_stream_fb_count = v;
        server.send(200, "application/json", String("{\"fb_count\":") + String(current_stream_fb_count) + String("}"));
        return;
    }
    server.send(400, "text/plain", "Bad Request");
}
// ----------------- WEB -----------------
void handle_root() {
    String html = "<h1>ESP32-S3 Gen4 Camera</h1>";
    html += "<p><a href='/capture'>Take High-Res Photo</a></p>";
    html += "<p><a href='/stream'>View 5 FPS Live Stream</a></p>";
    server.send(200, "text/html", html);
}

void handle_NotFound() {
    server.send(404, "text/plain", "Not Found");
}
// ------------------- ARDUINO -------------------
void setup() {
    Serial.begin(115200);
    Serial.println("\n--- BOOT ---");
    delay(900);

    if (!psramFound()) {
        Serial.println("❌ PSRAM NOT FOUND – CAMERA WILL NOT WORK");
        while (true) {
            delay(1000);
        }
    }
    Serial.printf("✅ PSRAM OK: %d bytes\n", ESP.getPsramSize());
    Serial.printf("PSRAM: %s\n", psramFound() ? "OK" : "FAIL");

    // Wait up to 5 seconds for Serial Monitor to open
    while (!Serial && millis() < 9000) {
        delay(10);
    }

    if(init_camera(stream_framesizes[stream_framesize_index], 25, current_stream_fb_count) != ESP_OK) {
        Serial.println("Camera failed to initialize!");
        while(true) delay(1000);
    }

    WiFi.begin(ssid, password);
    WiFi.setSleep(false); // Disables WiFi power saving for instant transmission
    Serial.print("Connecting to WiFi");
    while(WiFi.status() != WL_CONNECTED){
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("WiFi connected! IP: ");
    Serial.println(WiFi.localIP());

    server.on("/", handle_root);           // Now just the IP will work!
    server.on("/capture", handle_capture_page);
    server.on("/capture.jpg", handle_jpg);
    server.on("/quality", handle_quality_change);
    server.on("/stream", handle_stream_page); // Stream page with controls
    server.on("/stream.mjpg", handle_stream); // Raw MJPEG stream
    server.on("/stream_ctrl", handle_stream_ctrl);
    server.on("/send_email", handle_send_email);
    server.onNotFound(handle_NotFound);    // This stops the [E] handler not found error

    server.begin();
    Serial.println("HTTP server started.");
}

unsigned long lastLogTime = 0;
const unsigned long logInterval = 5000; // Print every 5 seconds

void loop() {
    server.handleClient();

    // Periodic Heartbeat Log
    if (millis() - lastLogTime >= logInterval) {
        lastLogTime = millis();
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.print("[LOG] IP: ");
            Serial.print(WiFi.localIP());
            Serial.print(" | Signal: ");
            Serial.print(WiFi.RSSI()); // Shows WiFi strength
            Serial.println(" dBm");
        } else {
            Serial.println("[LOG] WiFi Disconnected. Reconnecting...");
            // WiFi.begin(ssid, password); // Optional: auto-reconnect trigger
        }
    }
}
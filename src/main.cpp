#include "esp_camera.h"
#include <Arduino.h>
#include "esp_log.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <mbedtls/base64.h>
#include "credentials.h"
#include <Stepper.h>
#include <AccelStepper.h>

#include "FS.h"
#include "SPI.h"
#include "SD_MMC.h"

static const char* TAG = "camera";

#define BUFFER_SIZE 10
#define MAX_UXGA_SIZE (1024 * 1024) // 1MB per slot

// Array of 10 pointers, all starting as null
uint8_t* photo_buffer[BUFFER_SIZE] = { nullptr }; 
size_t photo_lengths[BUFFER_SIZE] = { 0 };
int current_buffer_idx = 0;
int total_buffered = 0;

// ------------------- SERVER -------------------
WebServer server(80);


// initialize the stepper library on pins 8 through 11:
AccelStepper stepper2(AccelStepper::FULL4WIRE, 1, 42, 2, 41);
AccelStepper stepper1(AccelStepper::FULL4WIRE, 47, 14, 21, 3);




void handle_show_last_photo() {
    // Calculate the index of the most recent photo
    // We subtract 1 from current_buffer_idx because the index moves forward AFTER saving
    int lastIdx = (current_buffer_idx - 1 + BUFFER_SIZE) % BUFFER_SIZE;

    if (total_buffered == 0 || photo_buffer[lastIdx] == nullptr) {
        server.send(404, "text/plain", "No image in buffer yet.");
        return;
    }

    size_t len = photo_lengths[lastIdx];
    uint8_t* buf = photo_buffer[lastIdx];

    server.setContentLength(len); 
    server.sendHeader("Content-Type", "image/jpeg");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    
    // Send the data from the specific buffer slot
    server.sendContent_P((const char *)buf, len);
}



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
    .jpeg_quality = 35,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY
};

// ----------------- RUNTIME SETTINGS -----------------
int current_capture_quality = 4;
const int capture_qualities[] = {0, 1, 2, 3, 4, 6, 8, 12, 18, 25, 35, 45, 63};
const int capture_qualities_len = sizeof(capture_qualities) / sizeof(capture_qualities[0]);

// Stream settings
framesize_t stream_framesizes[] = {FRAMESIZE_QVGA, FRAMESIZE_VGA, FRAMESIZE_SVGA, FRAMESIZE_XGA, FRAMESIZE_UXGA};
const char* stream_framesize_names[] = {"QVGA", "VGA", "SVGA", "XGA", "UXGA"};
const int stream_framesize_len = sizeof(stream_framesizes) / sizeof(stream_framesizes[0]);
int stream_framesize_index = 0; // default qVGA
int current_stream_fb_count = 3;


// ------------------- WIFI -------------------
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// ------------------- SMTP SETTINGS -----------------
const char* smtp_host = SMTP_HOST;
const int smtp_port = SMTP_PORT;
const char* smtp_user = SMTP_USER;
const char* smtp_pass = SMTP_PASS;
const char* smtp_from = SMTP_FROM;
const char* smtp_to = SMTP_TO;

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
    delay(50);    
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed: 0x%x", err);
        return err;
    }

    /*
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_exposure_ctrl(s, 0); // Manual Exposure
        s->set_aec_value(s, 800);   // Max Shutter (1200 is usually the limit)

        s->set_gain_ctrl(s, 0);     // Manual Gain (Turn off Auto)
        s->set_agc_gain(s, 15);      // Max Digital Boost (0 to 30)
        
        s->set_brightness(s, 2);     // Software offset: -2 to 2
        s->set_ae_level(s, 2);       // Target brightness boost
        s->set_dcw(s, 1);               // Digital Cleaning (helps with noise)
    } */


    /*

   sensor_t * s = esp_camera_sensor_get();
    if (s) {
        // 1. Exposure & Gain (Keep these manual to stop stripes)
        s->set_exposure_ctrl(s, 0); 
        s->set_aec_value(s, 600);    // Dropped from 800 to 600 (Reduces brightness)
        s->set_gain_ctrl(s, 0);
        s->set_agc_gain(s, 10);      // Dropped from 15 to 10 (Reduces "yellow" noise)

        // 2. The Color Fix (White Balance)
        s->set_whitebal(s, 1);       // Enable Auto White Balance
        s->set_awb_gain(s, 1);       // Enable White Balance Gain
        s->set_wb_mode(s, 0);        // 0: Auto, 1: Sunny, 2: Cloudy, 3: Office, 4: Home
        // If it's still yellow, try: s->set_wb_mode(s, 3); // "Office" mode cools down yellow light

        // 3. Remove the Over-Processing
        s->set_brightness(s, 0);     // Reset from 2 to 0 (This was causing the "washout")
        s->set_ae_level(s, 0);       // Reset from 2 to 0 (Not needed when using manual exposure)
        
        s->set_dcw(s, 1);            // Keep Digital Cleaning ON
        s->set_special_effect(s, 0); // Ensure no tint filters are active
    }

    */

    // s->set_aec2(s, 1);              // Enable Auto Exposure Control 2
    // s->set_agc_gain(s, 15);      // 3. Set a baseline Gain so it doesn't have to "guess"
    // --- THE STRIPE FIX ---    
    // s->set_antibanding(s, 1);       // 1 = 50Hz (Europe), 2 = 60Hz (USA)



    Serial.println("Camera initialized successfully!");
    return ESP_OK;
}



// ----------------- STREAM -----------------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

void handle_stream() {
    if(init_camera(stream_framesizes[stream_framesize_index], 45, current_stream_fb_count) != ESP_OK) { // VGA + medium quality
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


        if (server.client().available()) break;

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

        if (!client.connected()) {
        esp_camera_fb_return(fb);
        break; // EXIT THE LOOP IMMEDIATELY
        }

        unsigned long start = millis();
        while(millis() - start < 200) { // 5 FPS
            stepper1.run(); 
            stepper2.run();
            server.handleClient();
            yield();}
    }
}

void handle_back_to_menu() {
    // 1. Force the camera to 'sleep' to stop DMA background tasks
    // This prevents the "Capture camera init failed" on the next page
    esp_camera_deinit(); 
    
    // 2. Clear the current client socket
    WiFiClient client = server.client();
    if (client) {
        client.flush(); // Push out any remaining bytes
        client.stop();  // Close the door
    }

    // 3. Redirect the browser to the root "/"
    server.sendHeader("Location", "/");
    server.send(303); // 303 See Other is the standard for "Go elsewhere"
}



// ----------------- SINGLE CAPTURE -----------------
void handle_jpg() {
    // 1. Initialize camera for high-res UXGA
    if(init_camera(FRAMESIZE_UXGA, current_capture_quality, 3) != ESP_OK) {
        server.send(500, "text/plain", "Capture camera init failed");
        return;
    }

    // 2. Grab the frame from the sensor
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        server.send(500, "text/plain", "Camera Error");
        return;
    }

    // 1. Check if SD is still mounted
    if (!SD_MMC.cardSize()) {
        Serial.println("SD Lost! Attempting Re-mount...");
        SD_MMC.end(); 
        delay(100);
        SD_MMC.begin("/sdcard", true, false, 20000); // 10MHz for stability
    }

    // 2. Add a tiny delay to let the Camera power stabilize
    delay(100); 

    // 1. Generate filename based on current uptime (milliseconds)
    // Format: /img_123456.jpg
    // Change your save logic in handle_jpg to this:
    unsigned long uptime = millis();
    char path[32];
    // %010lu ensures the strings are always 10 characters: 0000001234
    sprintf(path, "/img_%010lu.jpg", uptime);

    // 2. Save to SD_MMC
    File file = SD_MMC.open(path, FILE_WRITE);
    if (file) {
        file.write(fb->buf, fb->len);
        file.close();
        Serial.printf("Saved: %s\n", path);
    } else {
        Serial.println("Failed to open file for writing - Card Error!");
    }
   
    // ------------------------------------------
    // 3. Update the global PSRAM buffer
    yield(); 
    

    // 
    // WRITE TO BUFFER
    //
    // 1. Check if the frame is within size limits
    if (fb->len <= MAX_UXGA_SIZE) {
    
    // 2. Ensure the specific slot in our array is allocated in PSRAM
    if (photo_buffer[current_buffer_idx] == nullptr) {
        photo_buffer[current_buffer_idx] = (uint8_t*)ps_malloc(MAX_UXGA_SIZE);
    }

    // 3. If allocation succeeded (or already existed), copy the data
    if (photo_buffer[current_buffer_idx] != nullptr) {
        memcpy(photo_buffer[current_buffer_idx], fb->buf, fb->len);
        photo_lengths[current_buffer_idx] = fb->len;

        Serial.printf("Saved to RAM Buffer slot: %d (%zu bytes)\n", current_buffer_idx, fb->len);

        // 4. Advance the index for the NEXT photo (0 to 9, then back to 0)
        current_buffer_idx = (current_buffer_idx + 1) % BUFFER_SIZE;
        
        // 5. Keep track of how many total images we've actually filled
        if (total_buffered < BUFFER_SIZE) total_buffered++;
    } else {
        Serial.println("PSRAM Error: Could not allocate buffer slot!");
    }
} else {
    Serial.println("Warning: Photo too large for 1MB buffer slot!");
}

    // 4. Send the headers manually
    server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.setContentLength(fb->len);
    server.send(200, "image/jpeg", "");

    // 5. Stream the actual bytes
    server.sendContent_P((const char *)fb->buf, fb->len);

    // 6. Release the hardware buffer
    esp_camera_fb_return(fb);

    // 7. Pulse motors
    stepper1.run();
    stepper2.run();
}




// --------- OPTIMIZED EMAIL FROM PSRAM BUFFER ---------
bool send_email_from_buffer(const char* to_addr, uint8_t* buf, size_t len) {
    if (!buf || len == 0) {
        Serial.println("[SMTP] Error: No data in buffer to send.");
        return false;
    }

    WiFiClientSecure client;
    client.setInsecure(); // Skip certificate verification for speed/simplicity
    
    Serial.println("[SMTP] Connecting to server...");
    if (!client.connect(smtp_host, smtp_port)) {
        Serial.println("[SMTP] Connection failed!");
        return false;
    }

    // Lambda helper to consume SMTP server replies
    auto clearResponse = [&]() {
        delay(150); 
        while (client.available()) {
            // Serial.print((char)client.read()); // Uncomment this line to see SMTP logs in Serial
            client.read();
        }
    };

    clearResponse();
    
    // --- 1. SMTP Handshake & Auth ---
    client.println("EHLO ESP32"); clearResponse();
    client.println("AUTH LOGIN"); clearResponse();
    // Encode credentials
    client.println(base64_encode((const uint8_t*)smtp_user, strlen(smtp_user))); clearResponse();
    client.println(base64_encode((const uint8_t*)smtp_pass, strlen(smtp_pass))); clearResponse();
    
    // --- 2. Set Up Mail Transaction ---
    client.print("MAIL FROM:<"); client.print(smtp_user); client.println(">"); clearResponse();
    client.print("RCPT TO:<"); client.print(to_addr); client.println(">"); clearResponse();
    client.println("DATA"); clearResponse();
    
    // --- 3. Write Email Headers ---
    String boundary = "esp32_m_boundary_987";
    client.print("From: "); client.println(smtp_user);
    client.print("To: "); client.println(to_addr);
    client.println("Subject: ESP32-S3 Captured Photo");
    client.println("MIME-Version: 1.0");
    client.printf("Content-Type: multipart/mixed; boundary=\"%s\"\r\n", boundary.c_str());
    client.println();
    
    // --- 4. Write Body Text ---
    client.printf("--%s\r\n", boundary.c_str());
    client.println("Content-Type: text/plain\r\n");
    client.println("Please find the requested photo from the ESP32-S3 attached.");
    client.println();
    
    // --- 5. Write Attachment Header ---
    client.printf("--%s\r\n", boundary.c_str());
    client.println("Content-Type: image/jpeg; name=\"photo.jpg\"");
    client.println("Content-Transfer-Encoding: base64");
    client.println("Content-Disposition: attachment; filename=\"photo.jpg\"\r\n");
    
    // --- 6. BUFFERED BASE64 ENCODING & SENDING ---
    Serial.println("[SMTP] Encoding and streaming Base64 data...");
    
    // Convert the PSRAM buffer to a Base64 string
    String img_b64 = base64_encode((const uint8_t*)buf, len);
    
    if (img_b64.length() == 0) {
        Serial.println("[SMTP] Base64 Encoding failed!");
        client.stop();
        return false;
    }

    const int CHUNK_SIZE = 4096; // 4KB Chunks for SSL efficiency
    int totalLen = img_b64.length();
    int spent = 0;

    while (spent < totalLen) {
        int toSend = (spent + CHUNK_SIZE < totalLen) ? CHUNK_SIZE : (totalLen - spent);
        
        // Write the chunk as raw bytes to keep SSL packets large
        client.write((const uint8_t*)(img_b64.c_str() + spent), toSend);
        
        spent += toSend;
        yield(); // Feed the watchdog to prevent reset
    }
    
    client.println(); // Final line ending for the Base64 block
    // ----------------------------------------------

    // --- 7. Finalize & Close ---
    client.printf("\r\n--%s--\r\n", boundary.c_str());
    client.println("."); // Required by SMTP to signal end of DATA
    clearResponse();
    
    client.println("QUIT");
    client.stop();
    
    Serial.println("[SMTP] Email sent successfully!");
    return true; // The function now always returns a value
}





String getMotorControls() {
    String html = "<div class='motor-container'>";
    html += "  <div class='dpad'>";
    html += "    <div style='grid-area: up;'><button onmousedown=\"go('forward')\" onmouseup=\"go('stop')\" ontouchstart=\"go('forward')\" ontouchend=\"go('stop')\">^</button></div>";
    html += "    <div style='grid-area: left;'><button onmousedown=\"go('left')\" onmouseup=\"go('stop')\" ontouchstart=\"go('left')\" ontouchend=\"go('stop')\"><</button></div>";
    html += "    <div style='grid-area: stop;'><button style='background:#ffcccc' onmousedown=\"go('stop')\" ontouchstart=\"go('stop')\">STOP</button></div>";
    html += "    <div style='grid-area: right;'><button onmousedown=\"go('right')\" onmouseup=\"go('stop')\" ontouchstart=\"go('right')\" ontouchend=\"go('stop')\">></button></div>";
    html += "    <div style='grid-area: down;'><button onmousedown=\"go('backward')\" onmouseup=\"go('stop')\" ontouchstart=\"go('backward')\" ontouchend=\"go('stop')\">V</button></div>";
    html += "  </div>";
    html += "</div>";
    return html;
}






void handle_capture_page() {
    // CRITICAL: Stop the camera before doing anything else
    esp_camera_deinit(); 
    delay(50); // Small breath for the hardware

    // Re-init for High Res
    if(init_camera(FRAMESIZE_UXGA, 4, 3) != ESP_OK) {
        server.send(500, "text/plain", "Camera Init Failed");
        return;
    }

    String motorControls = getMotorControls();
    String qualityStr = String(current_capture_quality);
    String timestamp = String(millis());

    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <style>
        body { font-family: sans-serif; text-align: center; background: #f9f9f9; }
        .control-panel { display: flex; justify-content: center; align-items: stretch; gap: 20px; flex-wrap: wrap; margin: 20px auto; max-width: 900px; }
        .dpad { display: grid; grid-template-areas: '. up .' 'left stop right' '. down .'; gap: 5px; }
        .camera-settings, .nav-panel { display: flex; flex-direction: column; gap: 10px; text-align: left; border-left: 2px solid #ddd; padding-left: 20px; justify-content: center; }
        button { padding: 12px; font-weight: bold; cursor: pointer; min-width: 60px; border-radius: 8px; border: 1px solid #ccc; }
        .action-btn { background: #fff3cd; width: 100%; }
    </style>
</head>
<body>
    <h1>ESP32-S3 Capture</h1>
    <div class='control-panel'>
        <div>
            <h4 style='margin:0 0 10px 0;'>Movement</h4>
            %MOTORS%
        </div>

        <div class='camera-settings'>
            <h4 style='margin:0 0 10px 0;'>Capture Settings</h4>
            <button class='action-btn' onclick="refreshImage()">CAPTURE NEW</button>
            <div>Quality: <span id='q'>%QUAL%</span><br>
                <button onclick="changeQuality('better')">+ better</button>
                <button onclick="changeQuality('less')">- less</button>
            </div>
        </div>

        <div class='nav-panel'>
            <h4 style='margin:0 0 10px 0;'>Navigation</h4>
            <button onclick="prepExit('/stream')" style="background:#c8e6c9;">LIVE STREAM</button>
            <button onclick="prepExit('/back')" style="background:#ffcdd2;">BACK TO MENU</button>
        </div>
    </div>

    <div style='margin-top: 20px;'>
        <img id="photo" src="/last_buffer.jpg?ts=%TS%" style="max-width:95%; border:3px solid #333; border-radius:10px;">
    </div>

    <script>
        // CRITICAL: Tells the ESP32 to release UXGA settings before moving
        function prepExit(url) {
            document.body.style.opacity = '0.5';
            document.body.style.pointerEvents = 'none';
            // Optional: call a 'reset' route if you have one, or just redirect
            window.location.href = url;
        }

        function go(dir){ fetch('/move?dir=' + dir); }
        
        function refreshImage(){ 
            const img=document.getElementById('photo'); 
            img.src='/capture.jpg?ts='+new Date().getTime(); 
        }
        
        function changeQuality(dir){ 
            fetch('/quality?dir='+dir).then(r=>r.json()).then(d=>{
                document.getElementById('q').innerText=d.quality; 
                refreshImage();
            }); 
        }
    
    </script>
</body>
</html>
)rawliteral";

    html.replace("%MOTORS%", motorControls);
    html.replace("%QUAL%", qualityStr);
    html.replace("%TS%", timestamp);

    server.send(200, "text/html", html); 
}



void handle_send_email() {
    uint8_t* temp_buf = nullptr;
    size_t fileSize = 0;
    String path = "";

    // 1. Check if we are sending from SD (Gallery) or RAM (Stream Button)
    if (server.hasArg("path")) {
        path = server.arg("path");
        File file = SD_MMC.open(path, FILE_READ);
        if (!file) {
            server.send(404, "text/plain", "File not found on SD card.");
            return;
        }
        fileSize = file.size();
        temp_buf = (uint8_t*)ps_malloc(fileSize);
        if (temp_buf) {
            file.read(temp_buf, fileSize);
        }
        file.close();
        Serial.printf("[Gallery] Sending %s from SD via Email...\n", path.c_str());
    } 
    else {
        // 2. No path? Send the LATEST photo from the RAM Circular Buffer
        int lastIdx = (current_buffer_idx - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        
        if (total_buffered > 0 && photo_buffer[lastIdx] != nullptr) {
            fileSize = photo_lengths[lastIdx];
            temp_buf = (uint8_t*)ps_malloc(fileSize); // Copy to temp so we don't block the buffer
            if (temp_buf) {
                memcpy(temp_buf, photo_buffer[lastIdx], fileSize);
            }
            Serial.println("[Stream] Sending latest RAM capture via Email...");
        } else {
            server.send(404, "text/plain", "No recent image in RAM buffer.");
            return;
        }
    }

    // 3. Final Check and Send
    if (temp_buf == nullptr) {
        server.send(500, "text/plain", "PSRAM Allocation Failed");
        return;
    }

    bool ok = send_email_from_buffer(smtp_to, temp_buf, fileSize);
    free(temp_buf); // Always free the temporary copy

    if (ok) {
        server.send(200, "text/plain", "Email sent successfully!");
    } else {
        server.send(500, "text/plain", "SMTP server rejected the email.");
    }
}






void handle_quality_change() {
    // Change current_capture_quality using capture_qualities array
    String dir = server.arg("dir");
    int idx = 0;
    for (int i = 0; i < capture_qualities_len; ++i) if (capture_qualities[i] == current_capture_quality) idx = i;

    if (dir == "less") {
        if (idx < capture_qualities_len - 1) idx++;
    } else if (dir == "better") {
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
    // CRITICAL: Stop the camera before doing anything else
    esp_camera_deinit(); 
    delay(50); // Small breath for the hardware
    
    // Re-init for High Res
    if(init_camera(FRAMESIZE_QVGA, 45, 3) != ESP_OK) {
        server.send(500, "text/plain", "Camera Init Failed");
        return;
    }

    // 1. Prepare the dynamic variables first
    String motorControls = getMotorControls();
    String fsName = String(stream_framesize_names[stream_framesize_index]);
    String fbCount = String(current_stream_fb_count);

    // 2. Use ONE giant Raw String with %PLACEHOLDERS%
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name='viewport' content='width=device-width, initial-scale=1.0'>
    <style>
        body { font-family: sans-serif; text-align: center; background: #f4f4f4; margin: 0; padding: 10px; }
        .main-container { display: flex; flex-wrap: wrap; justify-content: center; align-items: flex-start; gap: 20px; margin-top: 10px; }
        .video-pane { flex: 1; min-width: 320px; max-width: 800px; }
        .video-pane img { width: 100%; border: 3px solid #333; border-radius: 8px; background: #000; }
        .side-panel { width: 300px; display: flex; flex-direction: column; gap: 12px; background: #fff; padding: 15px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
        .dpad { display: grid; grid-template-areas: '. up .' 'left stop right' '. down .'; gap: 5px; justify-content: center; }
        button { padding: 12px; font-weight: bold; cursor: pointer; border-radius: 6px; border: 1px solid #ccc; background: #eee; }
        .capture-btn { background: #28a745; color: white; border: none; padding: 15px; font-size: 16px; width: 100%; }
        .capture-btn:disabled { background: #6c757d; cursor: wait; }
        .stream-opt { font-size: 0.9em; color: #555; text-align: left; border-top: 1px solid #eee; padding-top: 10px; }
        .nav-group { display: flex; flex-direction: column; gap: 8px; margin-top: 10px; }
    </style>
</head><body>
    <h2>ESP32-S3 Live Stream</h2>
    <div class='main-container'>
        <div class='video-pane'>
            <img src='/stream.mjpg' id='streamView'>
            <div id='statusMsg' style='margin-top:10px; font-weight:bold; color:#555;'>Mode: Streaming</div>
        </div>
        <div class='side-panel'>
            <div>
                <h4 style='margin:0 0 10px 0;'>Quick Action</h4>
                <button id='capBtn' class='capture-btn' onclick='takeSnapshot()'>SNAP HIGH-RES (UXGA)</button>
                <div id='capLink' style='font-size:0.85em; margin-top:5px;'></div>
                <button id='emailBtn' style='display:none; margin-top:8px; width:100%; background:#4CAF50; color:white; border:none; padding:8px; border-radius:4px; cursor:pointer;' onclick='emailLastPhoto()'>📧 Email Last Photo</button>
            </div>
            <div style='border-top: 1px solid #eee; padding-top: 10px;'>
                <h4 style='margin:0 0 10px 0;'>Movement</h4>
                %MOTORS%
            </div>
            <div class='stream-opt'>
                <p>Size: <b>%FS%</b> | Buffer: <b>%FB%</b></p>
                <button onclick="updateStream('framesize','down')">&#9664;</button>
                <button onclick="updateStream('framesize','up')">&#9654;</button>
                <button onclick="updateStream('fb_count','1')">FB:1</button>
                <button onclick="updateStream('fb_count','2')">FB:2</button>
                <button onclick="updateStream('fb_count','3')">FB:3</button>
            </div>
            <div class='nav-group' style="margin: 15px 0; display: flex; justify-content: center; gap: 10px;">
    <button onclick="cleanExit('/capture')" style="background:#c8e6c9; border:none; padding:12px 20px; border-radius:8px; cursor:pointer; font-weight:bold;">to CAPTURE PAGE</button>
    <button onclick="cleanExit('/gallery')" style="background:#bbdefb; border:none; padding:12px 20px; border-radius:8px; cursor:pointer; font-weight:bold;">to GALLERY</button>
    <button onclick="cleanExit('/')" style="background:#ffcdd2; border:none; padding:12px 20px; border-radius:8px; cursor:pointer; font-weight:bold;">TO MENU</button>
</div>
        </div>
    </div>
    <script>
        function cleanExit(targetUrl) {
            const view = document.getElementById('streamView');
            const status = document.getElementById('statusMsg');
            if(status) status.innerText = 'CLOSING CONNECTION...';
            view.src = '';
            window.stop();
            setTimeout(() => { window.location.href = targetUrl; }, 150);
        }

        function takeSnapshot() {
            const btn = document.getElementById('capBtn');
            const emailBtn = document.getElementById('emailBtn'); // Reference new button
            const view = document.getElementById('streamView');
            const status = document.getElementById('statusMsg');
            
            btn.disabled = true;
            emailBtn.style.display = 'none'; // Hide email button during new capture
            status.innerText = 'PAUSING STREAM...';
            view.src = ''; 
            
            setTimeout(() => {
                status.innerText = 'CAPTURING UXGA...';
                fetch('/capture.jpg').then(r => {
                    if(r.ok) {
                        status.innerText = 'SUCCESS!';
                        document.getElementById('capLink').innerHTML = '<a href="/last_buffer.jpg" target="_blank">View Photo</a>';
                        emailBtn.style.display = 'block'; // Show email button after success
                        emailBtn.innerText = '📧 Email Last Photo';
                        emailBtn.style.background = '#4CAF50';
                    }
                }).finally(() => {
                    setTimeout(() => { 
                        view.src = '/stream.mjpg'; 
                        btn.disabled = false; 
                        status.innerText = 'Mode: Streaming'; 
                    }, 1500);
                });
            }, 400);
        }

        // New Email Function
        function emailLastPhoto() {
            const eBtn = document.getElementById('emailBtn');
            eBtn.innerText = 'Sending...';
            eBtn.disabled = true;

            // We call the send_email route. Since it's the "last" one, 
            // your ESP32 handle_send_email logic should serve from the latest buffer slot.
            fetch('/send_email').then(r => {
                if(r.ok) {
                    eBtn.innerText = '✅ Sent!';
                    eBtn.style.background = '#2e7d32';
                } else {
                    eBtn.innerText = '❌ Error';
                    eBtn.style.background = '#c62828';
                }
                setTimeout(() => { 
                    eBtn.disabled = false; 
                    eBtn.innerText = '📧 Email Last Photo'; 
                    eBtn.style.background = '#4CAF50';
                }, 3000);
            }).catch(() => {
                eBtn.innerText = '❌ Failed';
                eBtn.disabled = false;
            });
        }

        function go(dir){ fetch('/move?dir=' + dir); }
        function updateStream(a, v){ 
            fetch('/stream_ctrl?action='+a+'&value='+v+'&dir='+v).then(() => { 
                if(a === 'framesize') location.reload();
            });
        }
    </script>
</body>
</html>
)rawliteral";

    // 3. Swap the placeholders with actual data
    html.replace("%MOTORS%", motorControls);
    html.replace("%FS%", fsName);
    html.replace("%FB%", fbCount);

    // 4. Send the cleaned-up string
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
    String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32-S3 Camera</title>
<style>
body {
    font-family: sans-serif; 
    text-align: center; 
    background: #f4f4f4; 
    margin: 0; 
    padding: 20px;
}
h1 {
    font-size: 1.4rem; 
    margin-bottom: 20px;
}
.nav-group {
    display: flex; 
    flex-direction: column;   /* stack vertically */
    align-items: center;      /* center buttons horizontally */
    gap: 12px; 
}
.nav-group a {
    display: inline-block; 
    padding: 12px 18px; 
    font-size: 1rem; 
    color: white; 
    text-decoration: none; 
    border-radius: 6px; 
    background: #28a745; 
    transition: background 0.2s;
}
.nav-group a:hover {
    background: #218838;
}
</style>
</head>
<body>
<h1>ESP32-S3 Gen4 Camera</h1>
<div class="nav-group">
    <a href='/capture'>Take High-Res Photo</a>
    <a href='/stream'>View 5 FPS Live Stream</a>
    <a href='/gallery'>View Gallery</a>
    <a href='/sdtest'>sdtest</a> 
</div> 
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
}



void handle_NotFound() {
    String path = server.uri();
    
    // Check if the request is for a file on the SD card
    if (SD_MMC.exists(path)) {
        File file = SD_MMC.open(path, "r");
        if (file) {
            String contentType = "application/octet-stream";
            if (path.endsWith(".jpg")) contentType = "image/jpeg";
            else if (path.endsWith(".txt")) contentType = "text/plain";
            else if (path.endsWith(".html")) contentType = "text/html";
            
            server.streamFile(file, contentType);
            file.close();
            return; // Successfully served the file!
        }
    }

    // If we get here, the file truly doesn't exist
    server.send(404, "text/plain", "404: Not Found");
}




// -------------------- MOTOR LOGIC --------------------
void handle_forward() {
  stepper1.move(2000); // Set target for 204 steps forward
  server.send(200, "text/plain", "Moving-forward...");
}

void handle_backward() {
  stepper1.move(-2000); // Set target for 204 steps forward
  server.send(200, "text/plain", "Moving-back...");
}


void handle_left() {
  stepper2.move(2000); // Set target for 204 steps forward
  server.send(200, "text/plain", "Moving-left...");
}

void handle_right() {
  stepper2.move(-2000); // Set target for 204 steps forward
  server.send(200, "text/plain", "Moving-right...");
}


void stop_motors() {
    stepper1.stop(); // Decelerates to stop
    stepper2.stop();
    // To kill power immediately so they don't get hot:
    stepper1.setCurrentPosition(stepper1.currentPosition()); 
    stepper2.setCurrentPosition(stepper2.currentPosition());
}


void handle_move() {
    if (!server.hasArg("dir")) {
        server.send(400, "text/plain", "Missing dir");
        return;
    }

    String direction = server.arg("dir");

    if (direction == "forward") {
        stepper1.move(2000);
    } else if (direction == "backward") {
        stepper1.move(-2000);
    } else if (direction == "left") {
        stepper2.move(2000);
    } else if (direction == "right") {
        stepper2.move(-2000);
    } else if (direction == "stop") {
        stop_motors();
    }

    server.send(200, "text/plain", "OK: " + direction);
}








void handle_gallery() {
    int page = 0; 
    if (server.hasArg("page")) page = server.arg("page").toInt();
    
    const int itemsPerPage = 15;
    int skipCount = page * itemsPerPage;

    String pageFiles[15]; 
    int totalJpgs = 0;
    int storedInPage = 0;

    // --- STEP 1: SCAN SD CARD FIRST ---
    // We must do this first so we know the 'totalJpgs' count
    File root = SD_MMC.open("/");
    if (!root) {
        server.send(500, "text/plain", "SD Card Error");
        return;
    }

    File file = root.openNextFile();
    while(file) {
        String name = String(file.name());
        if(name.endsWith(".jpg") || name.endsWith(".JPG")) {
            if (totalJpgs >= skipCount && storedInPage < itemsPerPage) {
                String fixedPath = name.startsWith("/") ? name : "/" + name;
                pageFiles[storedInPage] = fixedPath;
                storedInPage++;
            }
            totalJpgs++; 
        }
        file.close(); 
        file = root.openNextFile();
    }
    root.close();

    // --- STEP 2: NOW BUILD THE HTML (Now totalJpgs is accurate!) ---
    String html = "<html><head><title>S3 Gallery</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:sans-serif;text-align:center;background:#1a1a1a;color:white;}"
            ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:15px;padding:20px;}"
            "img{width:100%;border-radius:8px;height:180px;object-fit:cover;cursor:pointer;}"
            ".nav-bar{margin:20px 0;display:flex;justify-content:center;gap:10px;}"
            ".nav-btn{background:#00afff;color:white;padding:10px 20px;border:none;border-radius:5px;cursor:pointer;font-weight:bold;}"
            ".nav-btn:disabled{background:#444;color:#888;cursor:not-allowed;}</style></head><body>";
    
    html += "<h1>SD Gallery - Page " + String(page + 1) + "</h1>";

    // --- NEW NAVIGATION GROUP (Back to Stream/Menu) ---
    html += "<div class='nav-group'>";
    html += "  <button onclick=\"cleanExit('/stream')\" style=\"background:#e3f2fd;\">LIVE STREAM</button>";
    html += "  <button onclick=\"cleanExit('/capture')\" style=\"background:#c8e6c9;\">CAPTURE PAGE</button>";
    html += "  <button onclick=\"cleanExit('/')\" style=\"background:#ffcdd2;\">MENU</button>";
    html += "</div>";

    
    // Re-usable navigation snippet
    String navHtml = "<div class='nav-bar'>";
    if (page > 0) {
        navHtml += "<button class='nav-btn' onclick=\"cleanExit('/gallery?page=" + String(page - 1) + "')\">&larr; Previous</button>";
    } else {
        navHtml += "<button class='nav-btn' disabled>&larr; BEFORE</button>";
    }

    if (totalJpgs > (skipCount + itemsPerPage)) {
        navHtml += "<button class='nav-btn' onclick=\"cleanExit('/gallery?page=" + String(page + 1) + "')\">Next &rarr;</button>";
    } else {
        navHtml += "<button class='nav-btn' disabled>NEXT &rarr;</button>";
    }
    navHtml += "</div>";

    // Insert navigation at the TOP
    html += navHtml;

    html += "<div class='grid'>";
    for (int i = 0; i < storedInPage; i++) {
        html += "<div><a href='/saved_file?path=" + pageFiles[i] + "' target='_blank'>";
        html += "<img src='/saved_file?path=" + pageFiles[i] + "' loading='lazy'></a>";
        html += "<p style='font-size:10px;'>" + pageFiles[i].substring(1) + "</p>";
        html += "<button onclick=\"sendMail('" + pageFiles[i] + "', this)\" style='width:100%; cursor:pointer; background:#4CAF50; color:white; border:none; padding:5px; border-radius:3px;'>send via Email</button></div>";
    }
    html += "</div>";

    html += "<script>";
    html += "function sendMail(p, b) {";
    html += "  b.innerText='Sending...'; b.disabled=true;";
    html += "  fetch('/send_email?path='+p).then(r => {";
    html += "    if(r.ok) { b.innerText='Sent!'; b.style.background='#2e7d32'; }";
    html += "    else { b.innerText='Error'; b.style.background='#c62828'; }";
    html += "    setTimeout(() => { b.disabled=false; b.innerText='Email'; b.style.background='#4CAF50'; }, 3000);";
    html += "  }).catch(() => { b.innerText='Failed'; b.disabled=false; });";
    html += "}";

    html += "function cleanExit(targetUrl) {";
    html += "  window.stop();"; 
    html += "  window.location.href = targetUrl;";
    html += "}";
    html += "</script></body></html>";

    server.send(200, "text/html", html);
}


void verifySD() {
    File file = SD_MMC.open("/boot_log.txt", FILE_WRITE);
    if(file) {
        file.println("System rebooted successfully.");
        file.close();
        Serial.println("Verified: SD is writable.");
    }
}




void handle_sd_test() {
    // Change output to HTML format
    String output = "<html><head><title>SD Card Files</title>";
    output += "<style>body { font-family: monospace; background-color: #1a1a1a; color: #00ff00; padding: 20px; }";
    output += "a { color: #4db8ff; text-decoration: none; } a:hover { text-decoration: underline; }</style>";
    output += "</head><body>";
    output += "<h2>SD CARD FILE LIST:</h2>";
    output += "<hr>";

    File root = SD_MMC.open("/");
    if (!root) {
        server.send(500, "text/plain", "CRITICAL ERROR: Could not open SD Root");
        return;
    }

    File file = root.openNextFile();
    int count = 0;

    while(file) {
        String fileName = String(file.name());
        output += "[" + String(count) + "] ";
        
        if (file.isDirectory()) {
            output += "DIR : " + fileName + "<br>";
        } else {
            // Create a link to the file. 
            // This assumes you have a handler at /download?file=filename
            output += "FILE: <a href='/" + fileName + "' target='_blank'>" + fileName + "</a> (" + String(file.size()) + " bytes)<br>";
        }
        count++;
        file.close();
        file = root.openNextFile();
    }
    root.close();

    if (count == 0) output += "No files found on card.";
    
    output += "</body></html>";
    
    // Send as text/html instead of text/plain
    server.send(200, "text/html", output);
}



void handle_saved_file() {
    if (!server.hasArg("path")) {
        server.send(400, "text/plain", "Missing Path");
        return;
    }
    String path = server.arg("path");
    
    // Open the file from SD_MMC
    File file = SD_MMC.open(path, FILE_READ);
    if (!file) {
        server.send(404, "text/plain", "File Not Found");
        return;
    }

    // This streams the file directly to the browser
    server.streamFile(file, "image/jpeg");
    file.close();
}


// ------------------- ARDUINO -------------------
void setup() {
    Serial.begin(115200);
    
    // This waits up to 3 seconds for you to open the Serial Monitor window
    long start = millis();
    while (!Serial && millis() - start < 9000) {
        delay(10);
    }

    for (int i = 0; i < BUFFER_SIZE; i++) {
    photo_buffer[i] = nullptr;
    photo_lengths[i] = 0;
}

    Serial.println("\n--- BOOT ---");
    delay(900);

    SD_MMC.setPins(39, 38, 40); // CLK, CMD, D0 sdmmc_card_init failed (0x107).
    if (!SD_MMC.begin("/sdcard", true, false, 20000)) {
        Serial.println("SDMMC Mount Failed!");
    } else {
        Serial.println("SDMMC Mount SUCCESS!");
        // Now that it is mounted, you can call a simple test
        verifySD(); 
    }

    if (!psramFound()) {
        Serial.println("PSRAM NOT FOUND - CAMERA WILL NOT WORK");
        while (true) {
            delay(1000);
        }
    }
    Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());


    Serial.printf("✅ PSRAM OK: %d bytes\n", ESP.getPsramSize());
    Serial.printf("PSRAM: %s\n", psramFound() ? "OK" : "FAIL");

    // Wait up to 5 seconds for Serial Monitor to open
    while (!Serial && millis() < 5000) {
        delay(10);
    }

    Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());

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

    Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
    // Attempt to initialize camera until success
    while (init_camera(stream_framesizes[stream_framesize_index], 45, current_stream_fb_count) != ESP_OK) {
        Serial.println("Camera failed to initialize! Retrying in 1 second...");
        delay(1000);  // wait a bit before retry
    }
    Serial.println("Camera initialized successfully!");
    Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
    
    stepper1.setMaxSpeed(1000.0);
    stepper1.setAcceleration(500.0);

    stepper2.setMaxSpeed(1000.0);
    stepper2.setAcceleration(500.0);

    server.on("/", handle_root);           // Now just the IP will work!
    server.on("/capture", handle_capture_page);
    server.on("/capture.jpg", handle_jpg);
    server.on("/last_buffer.jpg", handle_show_last_photo); // Logic: JUST SEND BUF (No snap)    
    server.on("/quality", handle_quality_change);
    server.on("/stream", handle_stream_page); // Stream page with controls
    server.on("/stream.mjpg", handle_stream); // Raw MJPEG stream
    server.on("/stream_ctrl", handle_stream_ctrl);
    server.on("/send_email", handle_send_email);
    server.on("/move", handle_move);
    server.on("/back", handle_back_to_menu);
    server.on("/gallery", handle_gallery);
    server.on("/saved_file", handle_saved_file); 
    server.on("/sdtest", handle_sd_test);   
    server.onNotFound(handle_NotFound);    // This stops the [E] handler not found error

    server.begin();
    Serial.println("HTTP server started.");
    Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());

}

unsigned long lastLogTime = 0;
const unsigned long logInterval = 5000; // Print every 5 seconds

unsigned long lastMotorTime = 0;
const unsigned long motorInterval = 1000; // Print every 5 seconds


void loop() {
    server.handleClient(); // Handle web requests
    
    stepper1.run();
    stepper2.run();
    
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
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

static const char* TAG = "camera";

uint8_t* last_photo_buf = nullptr;
size_t last_photo_len = 0;


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
int current_capture_quality = 8;
const int capture_qualities[] = {0, 1, 2, 3, 4, 6, 8, 12, 18, 25, 35, 45, 63};
const int capture_qualities_len = sizeof(capture_qualities) / sizeof(capture_qualities[0]);

// Stream settings
framesize_t stream_framesizes[] = {FRAMESIZE_QVGA, FRAMESIZE_VGA, FRAMESIZE_SVGA, FRAMESIZE_XGA, FRAMESIZE_UXGA};
const char* stream_framesize_names[] = {"QVGA", "VGA", "SVGA", "XGA", "UXGA"};
const int stream_framesize_len = sizeof(stream_framesizes) / sizeof(stream_framesizes[0]);
int stream_framesize_index = 1; // default VGA
int current_stream_fb_count = 1;


// ------------------- WIFI -------------------
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// ------------------- SERVER -------------------
WebServer server(80);

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
    if(init_camera(FRAMESIZE_UXGA, current_capture_quality, 1) != ESP_OK) {
        server.send(500, "text/plain", "Capture camera init failed");
        return;
    }

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        server.send(500, "text/plain", "Camera capture failed");
        return;
    }

    // --- NEW: Save to Global Buffer in PSRAM ---
    if (last_photo_buf) free(last_photo_buf); // Clear old photo
    last_photo_buf = (uint8_t*)ps_malloc(fb->len); // Allocate in PSRAM
    if (last_photo_buf) {
        memcpy(last_photo_buf, fb->buf, fb->len);
        last_photo_len = fb->len;
    }
    // -------------------------------------------

    server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.setContentLength(fb->len);
    server.send(200, "image/jpeg", "");

    WiFiClient client = server.client();
    client.write(fb->buf, fb->len);

    esp_camera_fb_return(fb);
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
    String html = "<html><head><style>";
    // Layout CSS
    html += "body { font-family: sans-serif; text-align: center; }";
    html += ".control-panel { display: flex; justify-content: center; align-items: center; gap: 20px; flex-wrap: wrap; margin: 20px 0; }";
    html += ".dpad { display: grid; grid-template-areas: '. up .' 'left stop right' '. down .'; gap: 5px; }";
    html += ".camera-settings { display: flex; flex-direction: column; gap: 8px; text-align: left; border-left: 2px solid #ddd; padding-left: 20px; }";
    html += "button { padding: 12px; font-weight: bold; cursor: pointer; min-width: 55px; border-radius: 8px; border: 1px solid #ccc; }";
    html += ".action-btn { background: #fff3cd; width: 100%; }";
    html += "</style></head><body>";

    html += "<h1>ESP32-S3 Capture</h1>";

    // Main UI Row
    html += "<div class='control-panel'>";
    
    // Left: Motors
    html += getMotorControls();

    // Right: Camera Controls
    html += "  <div class='camera-settings'>";
    html += "    <button class='action-btn' onclick=\"refreshImage()\">CAPTURE NEW</button>";
    html += "    <div>Quality: <span id='q'>" + String(current_capture_quality) + "</span>";
    html += "      <button onclick=\"changeQuality('better')\">+ better</button>";
    html += "      <button onclick=\"changeQuality('less')\">- less</button>";
    html += "    </div>";
    html += "    <button id='emailBtn' onclick=\"sendEmail()\" style='background:#d1e7ff'>Send Email</button>";
    html += "    <span id='status' style='font-size: 0.8em;'></span>";
    html += "  </div>";
    
    html += "</div>"; // End control-panel

    html += "<p><a href='/'>[ Back to Menu ]</a></p>";
    html += "<img id=\"photo\" src=\"/capture.jpg?ts=" + String(millis()) + "\" style=\"max-width:95%; border:2px solid #333;\">";

    // JavaScript
    html += "<script>";
    html += "function go(dir){ fetch('/move?dir=' + dir); }";
    html += "function refreshImage(){ const img=document.getElementById('photo'); img.src='/capture.jpg?ts='+new Date().getTime(); }";
    html += "function changeQuality(dir){ fetch('/quality?dir='+dir).then(r=>r.json()).then(d=>{document.getElementById('q').innerText=d.quality; refreshImage();}); }";
    
    // Email Function
    html += "function sendEmail() {";
    html += "  const btn = document.getElementById('emailBtn'); const status = document.getElementById('status');";
    html += "  btn.disabled = true; status.innerText = '...sending...'; status.style.color = 'orange';";
    html += "  fetch('/send_email').then(r => r.ok ? (status.innerText='Sent', status.style.color='green') : r.text().then(t=>(status.innerText='Error '+t, status.style.color='red')))";
    html += "  .catch(e => status.innerText = 'Error').finally(() => btn.disabled = false);";
    html += "}";
    html += "</script></body></html>";

    server.send(200, "text/html", html);
}



void handle_send_email() {
    if (last_photo_buf == nullptr || last_photo_len == 0) {
        server.send(400, "text/plain", "No photo captured yet! Please click Capture New first.");
        return;
    }
    
    // We create a "fake" fb structure to pass to your email function
    // or simply modify your email function to accept raw buffer + len
    bool ok = send_email_from_buffer(smtp_to, last_photo_buf, last_photo_len);
    
    server.send(ok ? 200 : 500, "text/plain", ok ? "Email sent" : "Email send failed");
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
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>";
    html += "  body { font-family: sans-serif; text-align: center; background: #f4f4f4; margin: 0; padding: 10px; }";
    html += "  .main-container { display: flex; flex-wrap: wrap; justify-content: center; align-items: flex-start; gap: 20px; margin-top: 10px; }";
    html += "  .video-pane { flex: 1; min-width: 320px; max-width: 800px; }";
    html += "  .video-pane img { width: 100%; border: 3px solid #333; border-radius: 8px; background: #000; }";
    html += "  .side-panel { width: 300px; display: flex; flex-direction: column; gap: 15px; background: #fff; padding: 15px; border-radius: 10px; shadow: 0 4px 6px rgba(0,0,0,0.1); }";
    html += "  .dpad { display: grid; grid-template-areas: '. up .' 'left stop right' '. down .'; gap: 5px; justify-content: center; }";
    html += "  button { padding: 12px; font-weight: bold; cursor: pointer; border-radius: 6px; border: 1px solid #ccc; background: #eee; }";
    html += "  .stream-opt { font-size: 0.9em; color: #555; text-align: left; }";
    html += "  .nav-link { margin-top: 10px; display: block; color: #666; text-decoration: none; }";
    html += "</style></head><body>";

    html += "<h2>ESP32-S3 Live Stream</h2>";

    html += "<div class='main-container'>";
    
    // LEFT SIDE: Video
    html += "  <div class='video-pane'>";
    html += "    <img src='/stream.mjpg' id='streamView'>";
    html += "  </div>";

    // RIGHT SIDE: Controls
    html += "  <div class='side-panel'>";
    
    // Motor Controls (The D-Pad)
    html += "    <div style='border-bottom: 1px solid #eee; padding-bottom: 15px;'>";
    html += "      <h4 style='margin:0 0 10px 0;'>Movement</h4>";
    html +=        getMotorControls(); 
    html += "    </div>";

    // Stream Settings
    html += "    <div class='stream-opt'>";
    html += "      <h4 style='margin:0 0 10px 0;'>Stream Settings</h4>";
    html += "      <p>Size: <b id='fs'>" + String(stream_framesize_names[stream_framesize_index]) + "</b></p>";
    html += "      <button onclick=\"updateStream('framesize','down')\">&#9664;</button>";
    html += "      <button onclick=\"updateStream('framesize','up')\">&#9654;</button>";
    html += "      <p>Buffer (FB): <b id='fb'>" + String(current_stream_fb_count) + "</b></p>";
    html += "      <button onclick=\"updateStream('fb_count','1')\">FB:1</button>";
    html += "      <button onclick=\"updateStream('fb_count','2')\">FB:2</button>";
    html += "    </div>";

    html += "    <a href='/' class='nav-link'>&larr; Back to Menu</a>";
    html += "  </div>"; // End side-panel
    html += "</div>";   // End main-container

    // JavaScript for AJAX updates (No reload!)
    html += "<script>";
    html += "function go(dir){ fetch('/move?dir=' + dir); }";
    html += "function updateStream(action, value){ ";
    html += "  let url = '/stream_ctrl?action=' + action + (action==='framesize' ? '&dir=' : '&value=') + value;";
    html += "  fetch(url).then(() => {";
    html += "    if(action==='framesize') location.reload();"; // Resolution change usually needs a reconnect
    html += "    else fetch(location.href).then(r=>r.text()).then(h=>{";
    html += "      let parser = new DOMParser(); let doc = parser.parseFromString(h, 'text/html');";
    html += "      document.getElementById('fb').innerText = doc.getElementById('fb').innerText;";
    html += "    });";
    html += "  });";
    html += "}";
    html += "</script></body></html>";

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



// initialize the stepper library on pins 8 through 11:
AccelStepper stepper1(AccelStepper::FULL4WIRE, 1, 42, 2, 41);
AccelStepper stepper2(AccelStepper::FULL4WIRE, 48, 21, 47, 39);
//Stepper stepperOne(2048, 1, 42, 2, 41);
//Stepper stepperTwo(2048, 48, 21, 47, 39);


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
    if (server.hasArg("dir")) {
        String direction = server.arg("dir");
        
        if (direction == "forward")         handle_forward();
        else if (direction == "backward")  handle_backward();
        else if (direction == "left")  handle_left();
        else if (direction == "right") handle_right();
        else if (direction == "stop")  stop_motors();
        
        server.send(200, "text/plain", "OK: " + direction);
    } else {
        server.send(400, "text/plain", "Missing dir");
    }
}



// ------------------- ARDUINO -------------------
void setup() {
    Serial.begin(115200);
    last_photo_buf = nullptr; // Initialize to empty
    last_photo_len = 0;
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
    while (!Serial && millis() < 5000) {
        delay(10);
    }

    // Attempt to initialize camera until success
    while (init_camera(stream_framesizes[stream_framesize_index], 25, current_stream_fb_count) != ESP_OK) {
        Serial.println("Camera failed to initialize! Retrying in 1 second...");
        delay(1000);  // wait a bit before retry
    }

    Serial.println("Camera initialized successfully!");

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



    //stepperOne.setSpeed(5); stepperTwo.setSpeed(5);
    
    stepper1.setMaxSpeed(1000.0);
    stepper1.setAcceleration(500.0);

    stepper2.setMaxSpeed(1000.0);
    stepper2.setAcceleration(500.0);

    server.on("/", handle_root);           // Now just the IP will work!
    server.on("/capture", handle_capture_page);
    server.on("/capture.jpg", handle_jpg);
    server.on("/quality", handle_quality_change);
    server.on("/stream", handle_stream_page); // Stream page with controls
    server.on("/stream.mjpg", handle_stream); // Raw MJPEG stream
    server.on("/stream_ctrl", handle_stream_ctrl);
    server.on("/send_email", handle_send_email);
    server.on("/move", handle_move);
    server.onNotFound(handle_NotFound);    // This stops the [E] handler not found error

    server.begin();
    Serial.println("HTTP server started.");
}

unsigned long lastLogTime = 0;
const unsigned long logInterval = 5000; // Print every 5 seconds

unsigned long lastMotorTime = 0;
const unsigned long motorInterval = 1000; // Print every 5 seconds


void loop() {
    server.handleClient(); // Handle web requests
    
    stepper1.run();
    stepper2.run();
    
    /*
    stepperOne.step(204);
    delay(100);
    stepperTwo.step(204);
    delay(100);
    stepperOne.step(-204);
    delay(100);
    stepperTwo.step(-204);
    delay(100);
    */

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
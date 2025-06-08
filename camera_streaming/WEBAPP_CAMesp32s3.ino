#include <WiFi.h>
#include <WebServer.h>
#include "SPIFFS.h"
#include "esp_camera.h"

// Nombre de red WiFi que creará la ESP32
const char* ssid = "ESP32-Robot";
const char* password = "12345678";

// === Pines para XIAO ESP32S3 con cámara OV2640 ===
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39

#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13

// Servidor web en el puerto 80
WebServer server(80);

void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QVGA; // 320x240
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error al iniciar cámara: 0x%x\n", err);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!SPIFFS.begin(true)) {
  Serial.println("Error al montar SPIFFS");
  return;
}

initCamera();
// Inicia Access Point
WiFi.softAP(ssid, password);
Serial.println("Access Point iniciado");
Serial.print("IP: ");
Serial.println(WiFi.softAPIP());

  // Página web principal
  // Ruta para la página principal
server.on("/", HTTP_GET, []() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <meta name="viewport" content="width=device-width, initial-scale=1" />
      <style>
        body {
          margin: 0;
          font-family: sans-serif;
          background-color: white;
          color: black;
        }
        .cabecera {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 10px 20px;
        }
        .titulo {
          color: navy;
          font-weight: bold;
          font-size: 18px;
        }
        .logo {
          width: 60px;
          height: 60px;
          border-radius: 10px;
          background-color: #ccc;
        }
        .contenido {
          text-align: center;
          margin-top: 20px;
        }
        img.cam {
          width: 100%;
          max-width: 480px;
          border-radius: 10px;
        }
        #botones {
          margin-top: 20px;
        }
        .boton {
          width: 60px;
          height: 60px;
          margin: 10px;
          border-radius: 50%;
          background-color: #007BFF;
          color: white;
          font-size: 24px;
          border: none;
          cursor: pointer;
        }
      </style>
    </head>
    <body>
      <div class="cabecera">
        <div class="titulo">IR2162 - Diseño de Sistemas Empotrados y de Tiempo Real</div>
        <img src="/logo" class="logo" />
      </div>

      <div class="contenido">
        <img src="/stream" class="cam" />

        <div id="botones">
          <div><button class="boton" onclick="enviar('adelante')">&#8593;</button></div>
          <div>
            <button class="boton" onclick="enviar('izquierda')">&#8592;</button>
            <button class="boton" onclick="enviar('derecha')">&#8594;</button>
          </div>
          <div><button class="boton" onclick="enviar('atras')">&#8595;</button></div>
        </div>
      </div>

      <script>
        function enviar(direccion) {
          fetch('/' + direccion);
        }
      </script>
    </body>
    </html>
  )rawliteral";
  server.send(200, "text/html", html);
});

  // Rutas para cada botón
  server.on("/adelante", HTTP_GET, []() {
    Serial.println("Botón presionado: ADELANTE");
    server.send(200, "text/plain", "OK");
  });

  server.on("/atras", HTTP_GET, []() {
    Serial.println("Botón presionado: ATRÁS");
    server.send(200, "text/plain", "OK");
  });

  server.on("/izquierda", HTTP_GET, []() {
    Serial.println("Botón presionado: IZQUIERDA");
    server.send(200, "text/plain", "OK");
  });

  server.on("/derecha", HTTP_GET, []() {
    Serial.println("Botón presionado: DERECHA");
    server.send(200, "text/plain", "OK");
  });

  server.on("/logo", HTTP_GET, []() {
  File file = SPIFFS.open("/LOGO_UJI.jpg", "r");
  if (!file || file.isDirectory()) {
    server.send(404, "text/plain", "Logo no encontrado");
    return;
  }
  server.streamFile(file, "image/jpeg");
  file.close();
  });
  
  /*server.on("/fondo", HTTP_GET, []() {
    File file = SPIFFS.open("/ROBOT_PID.png", "r");
    server.streamFile(file, "image/png");
    file.close();
  });*/

  server.on("/stream", HTTP_GET, []() {
    WiFiClient client = server.client();

    String response = "HTTP/1.1 200 OK\r\n";
    response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    client.print(response);

    while (1) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) continue;

      client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", fb->len);
      client.write(fb->buf, fb->len);
      client.print("\r\n");

      esp_camera_fb_return(fb);
      if (!client.connected()) break;
    }
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "No encontrado");
  });

  // Inicia servidor
  server.begin();
  Serial.println("Servidor web iniciado");
}

void loop() {
  server.handleClient();
}

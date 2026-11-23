#include "web_page.h"
#include <Update.h>

// Create AsyncWebServer object on port 80
WebServer server(80);

void handleRoot(){
  server.send(200, "text/html", index_html); //Send web page
}

// ======== V2: OTA update ========
bool otaStarted = false;

const char otaPage[] PROGMEM = R"rawliteral(<!DOCTYPE html><html><head><meta charset="utf-8">
<title>WAVE ROVER OTA</title></head><body style="font-family:sans-serif">
<h3>WAVE ROVER firmware update (OTA)</h3>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin"> <input type="submit" value="Upload">
</form><p>Select firmware_v2.ino.bin. The car stops its motors and reboots when done.</p>
</body></html>)rawliteral";

void otaUploadHandler() {
  HTTPUpload &up = server.upload();
  // A ~1 MB upload runs inside one handleClient() call: keep the freeze
  // watchdog (motorSafetyTask) from rebooting the board mid-update.
  loopAliveMs = millis();
  if (up.status == UPLOAD_FILE_START) {
    turnCtrl.abort();
    straightCtrl.abort();
    setGoalSpeed(0, 0);
    otaStarted = Update.begin(UPDATE_SIZE_UNKNOWN);
    Serial.printf("OTA start: %s\n", up.filename.c_str());
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (otaStarted && Update.write(up.buf, up.currentSize) != up.currentSize) {
      Update.printError(Serial);
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (otaStarted && Update.end(true)) {
      Serial.printf("OTA done: %u bytes\n", up.totalSize);
    } else {
      Update.printError(Serial);
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    otaStarted = false;
    Serial.println("OTA aborted");
  }
}

void webCtrlServer(){
  server.on("/", handleRoot);

  server.on("/js", [](){
    String jsonCmdWebString = server.arg(0);
    deserializeJson(jsonCmdReceive, jsonCmdWebString);
    jsonCmdReceiveHandler();
    serializeJson(jsonInfoHttp, jsonFeedbackWeb);
    server.send(200, "text/plane", jsonFeedbackWeb);
    jsonFeedbackWeb = "";
    jsonInfoHttp.clear();
    jsonCmdReceive.clear();
  });

  // V2: over-the-air firmware update (no need to open the car for USB).
  //   browser : http://<car-ip>/update  (file picker page)
  //   CLI     : curl -F "firmware=@firmware_v2.ino.bin" http://<car-ip>/update
  // The image goes to the inactive OTA slot (app0/app1) and is only marked
  // bootable after a complete, verified write; an interrupted upload leaves the
  // running firmware untouched.
  server.on("/update", HTTP_GET, [](){
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", otaPage);
  });
  server.on("/update", HTTP_POST, [](){
    bool ok = otaStarted && !Update.hasError();
    server.sendHeader("Connection", "close");
    server.send(ok ? 200 : 500, "text/plain",
                ok ? "OK, rebooting\n" : String("FAIL: ") + Update.errorString() + "\n");
    otaStarted = false;
    if (ok) {
      delay(300);
      ESP.restart();
    }
  }, otaUploadHandler);

  // Start server
  server.begin();
  Serial.println("Server Starts.");
}

void initHttpWebServer(){
  webCtrlServer();
}
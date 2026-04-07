/*
 * provisioning.h — SoftAP captive portal + NVS para Aquantia (solo ESP32)
 *
 * Flujo de arranque:
 *   1. setup() comprueba si GPIO0 lleva >3s pulsado → factory reset NVS
 *   2. provisioning_load() carga credenciales desde NVS
 *   3. Si faltan credenciales → provisioning_start_ap() (bloquea hasta guardar)
 *   4. provisioning_start_ap() guarda en NVS y llama ESP.restart()
 *
 * Buffers globales rellenados por provisioning_load():
 *   prov_ssid, prov_password, prov_finca_id, prov_mqtt_token
 */
#pragma once

#ifndef ESP8266  // todo este archivo es solo ESP32

#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFi.h>

// ── Buffers globales de credenciales ─────────────────────────────────────────
char prov_ssid[64]       = "";
char prov_password[64]   = "";
char prov_finca_id[64]   = "";
char prov_mqtt_token[72] = "";  // token URL-safe (~43 chars) con margen

#define FACTORY_RESET_PIN 0   // GPIO0 — botón BOOT en casi todos los ESP32

// ── NVS ──────────────────────────────────────────────────────────────────────

static Preferences _prov_prefs;

/**
 * Carga credenciales desde NVS al espacio de nombre "aquantia".
 * Los buffers prov_* deben estar pre-rellenados con los valores de fallback
 * (secrets.h) antes de llamar a esta función; solo se sobreescriben si NVS
 * tiene un valor no vacío para esa clave.
 */
void provisioning_load() {
  _prov_prefs.begin("aquantia", /*readOnly=*/true);
  String s;
  s = _prov_prefs.getString("ssid", "");
  if (s.length() > 0) strlcpy(prov_ssid, s.c_str(), sizeof(prov_ssid));
  s = _prov_prefs.getString("password", "");
  if (s.length() > 0) strlcpy(prov_password, s.c_str(), sizeof(prov_password));
  s = _prov_prefs.getString("finca_id", "");
  if (s.length() > 0) strlcpy(prov_finca_id, s.c_str(), sizeof(prov_finca_id));
  s = _prov_prefs.getString("mqtt_token", "");
  if (s.length() > 0) strlcpy(prov_mqtt_token, s.c_str(), sizeof(prov_mqtt_token));
  _prov_prefs.end();
}

/** True si SSID y token están presentes (condición mínima para conectar). */
bool provisioning_has_credentials() {
  return strlen(prov_ssid) > 0 && strlen(prov_mqtt_token) > 0;
}

/** Guarda las credenciales en NVS y actualiza los buffers globales. */
void provisioning_save(const char* ssid, const char* pass,
                       const char* finca, const char* token) {
  _prov_prefs.begin("aquantia", /*readOnly=*/false);
  _prov_prefs.putString("ssid",       ssid);
  _prov_prefs.putString("password",   pass);
  _prov_prefs.putString("finca_id",   finca);
  _prov_prefs.putString("mqtt_token", token);
  _prov_prefs.end();
  strlcpy(prov_ssid,       ssid,  sizeof(prov_ssid));
  strlcpy(prov_password,   pass,  sizeof(prov_password));
  strlcpy(prov_finca_id,   finca, sizeof(prov_finca_id));
  strlcpy(prov_mqtt_token, token, sizeof(prov_mqtt_token));
}

/** Borra todas las claves de provisioning del NVS. */
void provisioning_clear() {
  _prov_prefs.begin("aquantia", /*readOnly=*/false);
  _prov_prefs.clear();
  _prov_prefs.end();
  prov_ssid[0] = prov_password[0] = prov_finca_id[0] = prov_mqtt_token[0] = '\0';
}

// ── HTML del captive portal ───────────────────────────────────────────────────

static const char _PROV_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Aquantia &mdash; Configuraci&oacute;n</title>
<style>
  *{box-sizing:border-box}
  body{font-family:system-ui,sans-serif;background:#f0f4f8;margin:0;
       padding:20px;color:#1a2a3a;min-height:100vh}
  .card{background:#fff;border-radius:16px;padding:24px;max-width:400px;
        margin:20px auto;box-shadow:0 2px 16px #0001}
  .logo{display:flex;align-items:center;gap:10px;margin-bottom:20px}
  .drop{width:36px;height:36px;background:#e8f5fd;border-radius:10px;
        display:flex;align-items:center;justify-content:center;font-size:20px}
  h2{margin:0;font-size:1.15rem;color:#0c8ecc}
  p.sub{margin:2px 0 0;font-size:.8rem;color:#888}
  label{display:block;font-size:.75rem;font-weight:700;color:#666;
        text-transform:uppercase;letter-spacing:.06em;margin:14px 0 4px}
  input{width:100%;border:1.5px solid #dde3ea;border-radius:10px;
        padding:10px 13px;font-size:.95rem;outline:none;color:#1a2a3a}
  input:focus{border-color:#0c8ecc;box-shadow:0 0 0 3px #0c8ecc22}
  .hint{font-size:.75rem;color:#aaa;margin:3px 0 0}
  button{margin-top:20px;width:100%;background:#0c8ecc;color:#fff;border:none;
         border-radius:10px;padding:13px;font-size:1rem;font-weight:700;
         cursor:pointer;letter-spacing:.01em}
  button:active{background:#0a7ab0}
  .sep{border:none;border-top:1px solid #eee;margin:18px 0}
  .mac{font-size:.75rem;color:#bbb;text-align:center}
</style>
</head>
<body>
<div class="card">
  <div class="logo">
    <div class="drop">&#128167;</div>
    <div><h2>Aquantia</h2><p class="sub">Configuraci&oacute;n del dispositivo</p></div>
  </div>
  <form method="POST" action="/save">
    <label>Red WiFi (SSID)</label>
    <input name="ssid" type="text" required autocomplete="off" placeholder="NombreDeRedWiFi">
    <label>Contrase&ntilde;a WiFi</label>
    <input name="password" type="password" autocomplete="off" placeholder="(vac&iacute;o si la red es abierta)">
    <label>Finca ID</label>
    <input name="finca_id" type="text" required autocomplete="off" placeholder="mi-finca">
    <p class="hint">Identificador &uacute;nico de tu instalaci&oacute;n (letras, n&uacute;meros y guiones).</p>
    <label>Token del dispositivo</label>
    <input name="mqtt_token" type="text" required autocomplete="off" placeholder="Copia el token de la etiqueta">
    <p class="hint">Impreso en la etiqueta del dispositivo. No lo compartas.</p>
    <button type="submit">Guardar y conectar &rarr;</button>
  </form>
  <hr class="sep">
  <p class="mac" id="mac"></p>
</div>
</body>
</html>
)rawliteral";

static const char _PROV_SAVED_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Aquantia &mdash; Listo</title>
<style>
  body{font-family:system-ui,sans-serif;background:#f0f4f8;margin:0;
       display:flex;align-items:center;justify-content:center;min-height:100vh}
  .card{background:#fff;border-radius:16px;padding:32px 28px;max-width:340px;
        text-align:center;box-shadow:0 2px 16px #0001}
  .ic{font-size:2.5rem;margin-bottom:12px}
  h2{color:#0c8ecc;margin:0 0 8px;font-size:1.2rem}
  p{color:#666;font-size:.9rem;margin:0}
</style>
</head>
<body>
<div class="card">
  <div class="ic">&#10003;</div>
  <h2>Configuraci&oacute;n guardada</h2>
  <p>El dispositivo se reiniciar&aacute; y conectar&aacute; a tu red WiFi en unos segundos.</p>
</div>
</body>
</html>
)rawliteral";

// ── SoftAP + captive portal ───────────────────────────────────────────────────

/**
 * Inicia el modo SoftAP con captive portal y espera a que el usuario
 * introduzca las credenciales. Al guardar, llama a ESP.restart().
 * Esta función NO retorna bajo operación normal.
 */
void provisioning_start_ap() {
  String macStr = WiFi.macAddress();  // "AA:BB:CC:DD:EE:FF"
  char ap_ssid[32];
  snprintf(ap_ssid, sizeof(ap_ssid), "Aquantia-%c%c%c%c%c%c",
           macStr[12], macStr[13], macStr[15], macStr[16],
           macStr[18], macStr[19]);

  Serial.printf("[PROV] Sin credenciales — AP: %s\n", ap_ssid);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid);
  delay(500);

  IPAddress apIP(192, 168, 4, 1);

  DNSServer  dns;
  dns.start(53, "*", apIP);

  WebServer server(80);

  server.on("/", HTTP_GET, [&server]() {
    server.send_P(200, "text/html; charset=utf-8", _PROV_HTML);
  });

  server.on("/save", HTTP_POST, [&server]() {
    String ssid  = server.arg("ssid");
    String pass  = server.arg("password");
    String finca = server.arg("finca_id");
    String token = server.arg("mqtt_token");

    if (ssid.isEmpty() || finca.isEmpty() || token.isEmpty()) {
      server.send(400, "text/plain", "Faltan campos obligatorios (ssid, finca_id, mqtt_token)");
      return;
    }

    provisioning_save(ssid.c_str(), pass.c_str(), finca.c_str(), token.c_str());
    server.send_P(200, "text/html; charset=utf-8", _PROV_SAVED_HTML);
    Serial.printf("[PROV] Guardado: ssid=%s finca=%s token=***\n",
                  ssid.c_str(), finca.c_str());
    delay(2000);
    ESP.restart();
  });

  // Redireccionar todo al portal (captive portal iOS/Android)
  server.onNotFound([&server]() {
    server.sendHeader("Location", "http://192.168.4.1", true);
    server.send(302, "text/plain", "");
  });

  server.begin();
  Serial.println("[PROV] Portal activo en http://192.168.4.1");
  Serial.println("[PROV] Esperando configuracion del usuario...");

  for (;;) {
    dns.processNextRequest();
    server.handleClient();
    delay(5);
  }
}

// ── Factory reset por pulsación larga de GPIO0 ───────────────────────────────

/**
 * Llamar al inicio de setup() (antes de Wire.begin).
 * Si GPIO0 (botón BOOT) se mantiene pulsado ≥3s, borra NVS y reinicia.
 * La duración evita resets accidentales durante el arranque normal.
 */
void provisioning_check_factory_reset() {
  pinMode(FACTORY_RESET_PIN, INPUT_PULLUP);
  if (digitalRead(FACTORY_RESET_PIN) != LOW) return;  // no pulsado

  Serial.print("[PROV] GPIO0 pulsado — manteniendo para factory reset");
  unsigned long t0 = millis();
  while (digitalRead(FACTORY_RESET_PIN) == LOW) {
    delay(100);
    Serial.print(".");
    if (millis() - t0 >= 3000) {
      Serial.println("\n[PROV] FACTORY RESET — borrando NVS...");
      provisioning_clear();
      Serial.println("[PROV] NVS borrado. Reiniciando en modo provisioning...");
      delay(300);
      ESP.restart();
    }
  }
  Serial.println(" (< 3s — cancelado)");
}

#endif  // !ESP8266

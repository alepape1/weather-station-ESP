# Aquantia firmware context for Copilot

This repository contains the ESP32 firmware for the Aquantia system.

Important context:
- The sibling repository app_meteo contains the dashboard, Flask API, database, and MQTT backend.
- This repo is only for firmware, provisioning, and flashing tools.
- There are two hardware profiles: METEO and IRRIGATION.
- secrets.h is for development-only local credentials and should not be treated as production configuration.

Guidelines:
- Stay focused on firmware files unless the user explicitly asks for full-stack integration.
- Prefer minimal, targeted reads.
- When discussing local testing, assume the app backend may run on localhost and MQTT may be non-TLS in development.

Token-saving rules:
- Do not scan the sibling app repository unless the task truly depends on it.
- Prefer the current firmware file and directly related headers or tools.
- Avoid reading generated assets or unrelated workspace content.

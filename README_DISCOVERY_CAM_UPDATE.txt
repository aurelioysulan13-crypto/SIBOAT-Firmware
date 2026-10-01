SIBOAT DISCOVERY UPDATE - MAIN + CAM INTEGRATED

FILES
1. A1a_ESP32_MAIN_FIRMWARE_SSID_CONFIGURED_DISCOVERY.ino
2. A1b_ESP32_CAM_FIRMWARE_SSID_CONFIGURED_DISCOVERY.ino
3. SIBOAT_DASHBOARD_UI_DISCOVERY_CAM.html
4. SIBOAT_Complete_Supabase_DISCOVERY_CAM.sql
5. build-siboat-firmware_DISCOVERY_CAM.yml

ARCHITECTURE
- MAIN publishes its current LAN IP to Supabase.
- CAM publishes its current LAN IP to Supabase.
- CAM also sends its current IP to MAIN through the existing HB UART packet.
- MAIN publishes the CAM IP as camera_ip in its discovery record.
- Dashboard discovers MAIN first and discovers CAM for direct MJPEG video.
- Static 10.90.102.50/10.90.102.51 and mDNS remain fallbacks.

DEPLOYMENT ORDER
1. Run the SQL in Supabase.
2. Replace the firmware source files in the repository.
3. Replace app/src/main/assets/index.html with the dashboard HTML.
4. Replace the firmware workflow.
5. Push to main and wait for both GitHub Actions jobs to succeed.
6. Flash MAIN and CAM all-in-one binaries.

IMPORTANT
The firmware has not been compiled by this environment. GitHub Actions is the final compiler verification.

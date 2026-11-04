#!/usr/bin/env python3
"""Static cross-file checks for the SIBOAT Main/CAM UART and CAM-link SQL contract."""

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]

MAIN = ROOT / (
    'firmware/main/SIBOAT_MAIN-ESP32_FIRMWARE_FINAL_BOSS/'
    'SIBOAT_MAIN-ESP32_FIRMWARE_FINAL_BOSS.ino'
)

CAM = ROOT / (
    'firmware/cam/SIBOAT_ESP32-CAM_FIRMWARE_FINAL_BOSS/'
    'SIBOAT_ESP32-CAM_FIRMWARE_FINAL_BOSS.ino'
)

SQL = ROOT / 'supabase/SIBOAT_CAM_TRANSPORT_AND_DISCOVERY.sql'

main = MAIN.read_text(encoding='utf-8')
cam = CAM.read_text(encoding='utf-8')
sql = SQL.read_text(encoding='utf-8')

errors = []
notes = []


def require(condition, message):
    if not condition:
        errors.append(message)


# UART baud rate and pin definitions.
main_baud = re.search(r'#define\s+UART_BAUD\s+(\d+)', main)
cam_baud = re.search(r'#define\s+UART_BAUD\s+(\d+)', cam)

main_tx = re.search(r'#define\s+PIN_UART_TX\s+(\d+)', main)
main_rx = re.search(r'#define\s+PIN_UART_RX\s+(\d+)', main)

cam_tx = re.search(r'#define\s+UART_TX_PIN\s+(\d+)', cam)
cam_rx = re.search(r'#define\s+UART_RX_PIN\s+(\d+)', cam)

require(
    all((main_baud, cam_baud, main_tx, main_rx, cam_tx, cam_rx)),
    'Could not read all UART pin/baud definitions.'
)

if all((main_baud, cam_baud, main_tx, main_rx, cam_tx, cam_rx)):
    require(
        main_baud.group(1) == cam_baud.group(1),
        'Main and CAM UART baud rates differ.'
    )

    notes.append(
        f"UART: {main_baud.group(1)} baud; "
        f"Main TX{main_tx.group(1)} -> CAM RX{cam_rx.group(1)}, "
        f"CAM TX{cam_tx.group(1)} -> Main RX{main_rx.group(1)} "
        "(cross-connect required)."
    )


# UART message compatibility.
for token, description in [
    ('<CAT,', 'CAM classification frame'),
    ('<REJ,', 'CAM rejection frame'),
    ('<HB,', 'CAM heartbeat frame'),
    ('<ST,', 'Main status frame'),
    ('ACK%d', 'Main classification acknowledgement'),
    ('<ACK,', 'CAM component acknowledgement parser')
]:
    source = (
        cam
        if token in ('<CAT,', '<REJ,', '<HB,', '<ACK,')
        else main
    )

    require(
        token in source,
        f'Missing expected UART protocol token: {description}.'
    )


require(
    'strcmp(line, txExpect) == 0' in cam
    and 'strcpy(txExpect, "REJECT")' in cam,
    'CAM ACK expectations are not defined for both CAT and REJ messages.'
)

require(
    'strcmp(tok[0], "CAT")' in main
    and 'strcmp(tok[0], "REJ")' in main
    and 'strcmp(tok[0], "HB")' in main,
    'Main UART parser does not handle all CAM message types.'
)


# CAM link JSON keys must be accepted by the SQL RPC signature.
payload_start = cam.find('static void publishCamLink()')

payload_end = cam.find(
    'static void discoveryPublishAll',
    payload_start
)

payload_fn = (
    cam[payload_start:payload_end]
    if payload_start >= 0 and payload_end > payload_start
    else ''
)

keys = set(
    re.findall(
        r'\\"(p_[a-z0-9_]+)\\"',
        payload_fn
    )
)

sig_match = re.search(
    r'create\s+or\s+replace\s+function\s+public\.'
    r'seabot_register_cam_link\s*\((.*?)\)\s*returns',
    sql,
    re.I | re.S
)

require(
    bool(payload_fn),
    'Could not locate CAM publishCamLink() payload builder.'
)

require(
    bool(sig_match),
    'SQL migration does not define seabot_register_cam_link().'
)

if sig_match:
    params = set(
        re.findall(
            r'\b(p_[a-z0-9_]+)\s+(?:text|integer|boolean|bigint)',
            sig_match.group(1),
            re.I
        )
    )

    missing = sorted(keys - params)

    require(
        not missing,
        'CAM sends CAM-link RPC parameter(s) not declared in SQL: '
        + ', '.join(missing)
    )

    notes.append(
        'CAM link RPC: payload keys match the SQL function signature.'
    )


# Camera endpoint and port consistency.
require(
    '#define CAM_HTTP_PORT         STREAM_PORT' in cam,
    'CAM HTTP and stream port are not tied together.'
)

for route in ('/stream', '/snapshot', '/api/status'):
    require(
        route in cam,
        f'CAM firmware does not implement/reference expected route {route}.'
    )

require(
    "'SIBOAT-CAM'" in sql and "'SIBOAT-CAM" in sql,
    'SQL camera device ID validation/default is missing.'
)


# Identify firmware RPC dependencies not created in this SQL migration.
used = set(
    re.findall(
        r'/rest/v1/rpc/([A-Za-z0-9_]+)',
        main + cam
    )
)

defined = set(
    re.findall(
        r'create\s+or\s+replace\s+function\s+public\.([A-Za-z0-9_]+)',
        sql,
        re.I
    )
)

external = sorted(used - defined)

if external:
    notes.append(
        'Firmware RPCs not created by the supplied SQL migration '
        '(must already exist in the target Supabase project): '
        + ', '.join(external)
        + '.'
    )


# Print the audit result.
if errors:
    print('SIBOAT INTERFACE AUDIT: FAIL')

    for item in errors:
        print('ERROR:', item)

    for item in notes:
        print('NOTE:', item)

    sys.exit(1)


print('SIBOAT INTERFACE AUDIT: PASS')

for item in notes:
    print('NOTE:', item)

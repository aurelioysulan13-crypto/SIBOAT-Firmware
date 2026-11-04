-- SIBOAT connection type (Wi-Fi or Bluetooth) preference.
-- Run ONCE in the Supabase SQL editor of the SIBOAT project. Safe to run again.
-- The dashboard writes the choice with seabot_set_transport(); the Main ESP32 reads it with
-- seabot_get_transport() while it is on Wi-Fi with internet. Without this script everything still
-- works over the local Wi-Fi link; only "select Bluetooth while the dashboard is not linked to the board" needs it.

create table if not exists public.seabot_transport_preference (
  device_id         text primary key,
  desired_transport text not null default 'wifi' check (desired_transport in ('wifi', 'bluetooth')),
  updated_at        timestamptz not null default now()
);

-- Row level security on, and no table policies on purpose: the table is reachable only through the two functions below.
alter table public.seabot_transport_preference enable row level security;

create or replace function public.seabot_set_transport(p_device_id text, p_transport text)
returns text
language plpgsql
security definer
set search_path = public
as $$
begin
  if p_device_id is null or length(p_device_id) = 0 or length(p_device_id) > 64 then
    raise exception 'invalid device id';
  end if;
  if p_transport is null or p_transport not in ('wifi', 'bluetooth') then
    raise exception 'invalid transport';
  end if;
  insert into public.seabot_transport_preference (device_id, desired_transport, updated_at)
  values (p_device_id, p_transport, now())
  on conflict (device_id) do update
    set desired_transport = excluded.desired_transport,
        updated_at = now();
  return p_transport;
end;
$$;

create or replace function public.seabot_get_transport(p_device_id text)
returns text
language sql
security definer
stable
set search_path = public
as $$
  select desired_transport from public.seabot_transport_preference where device_id = p_device_id;
$$;

revoke all on function public.seabot_set_transport(text, text) from public;
revoke all on function public.seabot_get_transport(text) from public;
grant execute on function public.seabot_set_transport(text, text) to anon, authenticated;
grant execute on function public.seabot_get_transport(text) to anon, authenticated;

-- Quick check (optional):
--   select public.seabot_set_transport('SIBOAT-MAIN', 'wifi');
--   select public.seabot_get_transport('SIBOAT-MAIN');   -- returns: wifi

-- =====================================================================================================
-- PART 2: ESP32-CAM connection link (Wi-Fi name, IP address, stream link, signal).
-- The CAM firmware 1.5.0-camlink-ota calls seabot_register_cam_link() every 10 s. The dashboard reads the
-- record with seabot_get_cam_link() and shows it in the Connection health card.
-- Run once in the Supabase SQL editor. Safe to run again. PART 1 above and the existing
-- seabot_register_device registry are not changed.
-- Anyone holding the publishable key can call these functions, so the register function only accepts
-- a private LAN address (10.x, 172.16 to 172.31.x, 192.168.x) and links that point at that same address.
-- =====================================================================================================

create table if not exists public.seabot_cam_link (
  device_id        text primary key,
  ssid             text,
  ip               text not null,
  gateway          text,
  subnet           text,
  rssi             integer,
  mac              text,
  hostname         text,
  http_port        integer not null default 81,
  stream_url       text not null,
  snapshot_url     text,
  status_url       text,
  firmware_version text,
  boot_id          text,
  fps              integer,
  camera_ok        boolean,
  model_ok         boolean,
  uptime_s         bigint,
  first_seen       timestamptz not null default now(),
  last_seen        timestamptz not null default now()
);

-- Row level security on, and no table policies on purpose: the table is reachable only through the functions below.
alter table public.seabot_cam_link enable row level security;

create or replace function public.seabot_register_cam_link(
  p_device_id        text,
  p_ip               text,
  p_ssid             text    default null,
  p_gateway          text    default null,
  p_subnet           text    default null,
  p_rssi             integer default null,
  p_mac              text    default null,
  p_hostname         text    default null,
  p_http_port        integer default 81,
  p_stream_url       text    default null,
  p_snapshot_url     text    default null,
  p_status_url       text    default null,
  p_firmware_version text    default null,
  p_boot_id          text    default null,
  p_fps              integer default null,
  p_camera_ok        boolean default null,
  p_model_ok         boolean default null,
  p_uptime_s         bigint  default null
)
returns void
language plpgsql
security definer
set search_path = public
as $$
declare
  v_ip       inet;
  v_host     text;
  v_port     integer;
  v_base     text;
  v_stream   text;
  v_snapshot text;
  v_status   text;
begin
  if p_device_id is null or p_device_id !~ '^SIBOAT-CAM[A-Za-z0-9_-]{0,16}$' then
    raise exception 'invalid camera device id';
  end if;

  begin
    v_ip := trim(p_ip)::inet;
  exception when others then
    raise exception 'invalid camera ip';
  end;
  if family(v_ip) <> 4 or masklen(v_ip) <> 32
     or not (v_ip <<= inet '10.0.0.0/8' or v_ip <<= inet '172.16.0.0/12' or v_ip <<= inet '192.168.0.0/16') then
    raise exception 'camera ip must be a private LAN address';
  end if;

  v_host := host(v_ip);
  v_port := coalesce(p_http_port, 81);
  if v_port < 1 or v_port > 65535 then
    raise exception 'invalid http port';
  end if;
  v_base := 'http://' || v_host || ':' || v_port;

  -- The links must point at the camera itself. If the firmware sends none, the standard ones are built here.
  v_stream   := coalesce(nullif(trim(p_stream_url), ''),   v_base || '/stream');
  v_snapshot := coalesce(nullif(trim(p_snapshot_url), ''), v_base || '/snapshot');
  v_status   := coalesce(nullif(trim(p_status_url), ''),   v_base || '/api/status');
  if not (starts_with(v_stream,   v_base || '/') and length(v_stream)   <= 200 and v_stream   ~ '^[A-Za-z0-9:/._?=&%-]+$')
  or not (starts_with(v_snapshot, v_base || '/') and length(v_snapshot) <= 200 and v_snapshot ~ '^[A-Za-z0-9:/._?=&%-]+$')
  or not (starts_with(v_status,   v_base || '/') and length(v_status)   <= 200 and v_status   ~ '^[A-Za-z0-9:/._?=&%-]+$') then
    raise exception 'camera links must point at the camera address';
  end if;

  insert into public.seabot_cam_link (
    device_id, ssid, ip, gateway, subnet, rssi, mac, hostname, http_port,
    stream_url, snapshot_url, status_url, firmware_version, boot_id,
    fps, camera_ok, model_ok, uptime_s, first_seen, last_seen
  ) values (
    p_device_id,
    left(regexp_replace(coalesce(p_ssid, ''), '[[:cntrl:]]', '', 'g'), 64),
    v_host,
    case when p_gateway ~ '^[0-9]{1,3}(\.[0-9]{1,3}){3}$' then p_gateway end,
    case when p_subnet  ~ '^[0-9]{1,3}(\.[0-9]{1,3}){3}$' then p_subnet end,
    case when p_rssi between -127 and 0 then p_rssi end,
    case when p_mac ~ '^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$' then upper(p_mac) end,
    case when p_hostname ~ '^[A-Za-z0-9.-]{1,63}$' then p_hostname end,
    v_port,
    v_stream, v_snapshot, v_status,
    left(p_firmware_version, 40),
    left(p_boot_id, 64),
    case when p_fps between 0 and 120 then p_fps end,
    p_camera_ok,
    p_model_ok,
    case when p_uptime_s >= 0 then p_uptime_s end,
    now(), now()
  )
  on conflict (device_id) do update set
    ssid             = excluded.ssid,
    ip               = excluded.ip,
    gateway          = excluded.gateway,
    subnet           = excluded.subnet,
    rssi             = excluded.rssi,
    mac              = excluded.mac,
    hostname         = excluded.hostname,
    http_port        = excluded.http_port,
    stream_url       = excluded.stream_url,
    snapshot_url     = excluded.snapshot_url,
    status_url       = excluded.status_url,
    firmware_version = excluded.firmware_version,
    boot_id          = excluded.boot_id,
    fps              = excluded.fps,
    camera_ok        = excluded.camera_ok,
    model_ok         = excluded.model_ok,
    uptime_s         = excluded.uptime_s,
    last_seen        = now();
end;
$$;

-- The dashboard calls this one. It returns one JSON object; "online" is true while the CAM reported in the last 35 seconds.
create or replace function public.seabot_get_cam_link(p_device_id text default 'SIBOAT-CAM')
returns json
language plpgsql
security definer
stable
set search_path = public
as $$
declare
  r     public.seabot_cam_link%rowtype;
  v_age integer;
begin
  select * into r
  from public.seabot_cam_link
  where device_id = coalesce(nullif(trim(p_device_id), ''), 'SIBOAT-CAM');

  if not found then
    return json_build_object('found', false);
  end if;

  v_age := greatest(0, floor(extract(epoch from (now() - r.last_seen)))::integer);

  return json_build_object(
    'found',           true,
    'deviceId',        r.device_id,
    'ssid',            r.ssid,
    'ip',              r.ip,
    'gateway',         r.gateway,
    'subnet',          r.subnet,
    'rssi',            r.rssi,
    'mac',              r.mac,
    'hostname',        r.hostname,
    'httpPort',        r.http_port,
    'streamUrl',       r.stream_url,
    'snapshotUrl',     r.snapshot_url,
    'statusUrl',       r.status_url,
    'firmwareVersion', r.firmware_version,
    'bootId',          r.boot_id,
    'fps',             r.fps,
    'cameraOk',        r.camera_ok,
    'modelOk',         r.model_ok,
    'uptimeSec',       r.uptime_s,
    'firstSeen',       r.first_seen,
    'lastSeen',        r.last_seen,
    'ageSeconds',      v_age,
    'online',          v_age <= 35
  );
end;
$$;

-- Same access model as PART 1: only the two functions are callable, with the publishable (anon) key.
do $$
declare f record;
begin
  for f in
    select p.oid::regprocedure as sig
    from pg_proc p
    join pg_namespace n on n.oid = p.pronamespace
    where n.nspname = 'public'
      and p.proname in ('seabot_register_cam_link', 'seabot_get_cam_link')
  loop
    execute format('revoke all on function %s from public', f.sig);
    execute format('grant execute on function %s to anon, authenticated', f.sig);
  end loop;
end;
$$;

notify pgrst, 'reload schema';

-- Quick check (optional), after the CAM has booted with firmware 1.5.0-camlink-ota and joined Wi-Fi:
--   select public.seabot_get_cam_link();
--   returns: {"found": true, "ssid": "...", "ip": "10.90.102.51", "streamUrl": "http://10.90.102.51:81/stream", "online": true, ...}
-- Check without the camera (writes a test row that the real CAM overwrites on its next report):
--   select public.seabot_register_cam_link('SIBOAT-CAM', '10.90.102.51', 'TEST WIFI');
--   select public.seabot_get_cam_link();

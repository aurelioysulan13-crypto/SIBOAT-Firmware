-- SIBOAT COMPLETE SUPABASE DATABASE
-- 2026-09
--
-- This file combines the supplied base SIBOAT/TRI-SEABOT database setup with
-- the supplied SIBOAT analytics add-on. Existing seabot_* table/function names
-- are intentionally retained so the current dashboard remains compatible.
--
-- IMPORTANT:
-- - Run this in the Supabase SQL editor for the SIBOAT project.
-- - Disable Auth > Providers > Email > Confirm email if username-only signup
--   is required by the current dashboard build.
-- - Never place a service_role/secret key in HTML or ESP32 firmware.
-- - The ESP32 operates locally and does not connect to Supabase directly.

-- SIBOAT — COMPLETE NEW-DATABASE SUPABASE SETUP
-- 2026-09-17
-- Project: fgvpopxbbgtilbzwsopv
-- This file is designed for a NEW Supabase project and the companion
-- SIBOAT HTML build supplied with this package.
--
-- IMPORTANT AUTH SETTING (Dashboard):
-- Authentication -> Providers -> Email -> disable "Confirm email".
-- Supabase documents that disabling Confirm Email returns a session directly
-- after signUp(); this HTML intentionally uses username-only UI with an
-- internal Auth email. SQL cannot change that hosted-project Auth setting.
--
-- SECURITY:
-- - Never put a service_role/secret key in the HTML.
-- - The publishable key belongs in the browser.
-- - Lead-only authorization is enforced in database functions/RLS.
-- - Lead account deletion deletes auth.users directly from this privileged
--   database function; the public profile cascades from auth.users.

BEGIN;

CREATE EXTENSION IF NOT EXISTS pgcrypto;

CREATE TABLE IF NOT EXISTS public.seabot_accounts (
  id uuid PRIMARY KEY REFERENCES auth.users(id) ON DELETE CASCADE,
  name text NOT NULL,
  username text NOT NULL,
  email text NOT NULL,
  role text NOT NULL DEFAULT 'visitor' CHECK (role IN ('developer','operator','visitor')),
  status text NOT NULL DEFAULT 'active' CHECK (status IN ('active','pending','denied','suspended','blocked')),
  is_lead boolean NOT NULL DEFAULT false,
  priority integer NOT NULL DEFAULT 0 CHECK (priority BETWEEN 0 AND 1000),
  priority_rank integer NOT NULL DEFAULT 10 CHECK (priority_rank BETWEEN 1 AND 10),
  comments text NOT NULL DEFAULT '',
  intro_seen boolean NOT NULL DEFAULT false,
  created_at timestamptz NOT NULL DEFAULT now(),
  last_seen_at timestamptz
);

CREATE UNIQUE INDEX IF NOT EXISTS seabot_accounts_username_ci
  ON public.seabot_accounts (lower(username));
CREATE UNIQUE INDEX IF NOT EXISTS seabot_accounts_email_ci
  ON public.seabot_accounts (lower(email));
CREATE UNIQUE INDEX IF NOT EXISTS seabot_accounts_one_lead
  ON public.seabot_accounts (is_lead)
  WHERE is_lead = true;

CREATE TABLE IF NOT EXISTS public.seabot_access_requests (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  account_id uuid NOT NULL REFERENCES public.seabot_accounts(id) ON DELETE CASCADE,
  requested_role text NOT NULL CHECK (requested_role IN ('developer','operator')),
  status text NOT NULL DEFAULT 'pending' CHECK (status IN ('pending','accepted','denied')),
  response_message text NOT NULL DEFAULT '',
  reviewed_by uuid REFERENCES public.seabot_accounts(id) ON DELETE SET NULL,
  created_at timestamptz NOT NULL DEFAULT now(),
  updated_at timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX IF NOT EXISTS seabot_access_requests_account_idx
  ON public.seabot_access_requests(account_id, created_at DESC);
CREATE INDEX IF NOT EXISTS seabot_access_requests_status_idx
  ON public.seabot_access_requests(status, created_at DESC);

CREATE TABLE IF NOT EXISTS public.seabot_messages (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  sender_id uuid NOT NULL REFERENCES public.seabot_accounts(id) ON DELETE CASCADE,
  recipient_id uuid NOT NULL REFERENCES public.seabot_accounts(id) ON DELETE CASCADE,
  body text NOT NULL CHECK (length(trim(body)) > 0),
  created_at timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX IF NOT EXISTS seabot_messages_sender_idx ON public.seabot_messages(sender_id, created_at);
CREATE INDEX IF NOT EXISTS seabot_messages_recipient_idx ON public.seabot_messages(recipient_id, created_at);

CREATE TABLE IF NOT EXISTS public.seabot_app_settings (
  id integer PRIMARY KEY CHECK (id = 1),
  settings jsonb NOT NULL DEFAULT '{}'::jsonb,
  updated_by uuid REFERENCES public.seabot_accounts(id) ON DELETE SET NULL,
  updated_at timestamptz NOT NULL DEFAULT now()
);

INSERT INTO public.seabot_app_settings(id, settings)
VALUES (
  1,
  '{
    "theme":"marine",
    "defaultSensitivity":80,
    "defaultSpeed":75,
    "defaultConveyorSpeed":65,
    "returnToCenter":true,
    "sound":true,
    "haptics":true,
    "animations":true,
    "smootherAnimations":true,
    "refinedLook":true,
    "performanceMode":false,
    "compact":false,
    "connectionTimeout":3,
    "autoReconnect":true,
    "forceDemo":false,
    "shutdownConfirm":true,
    "estopConfirm":true,
    "controlTimeout":60
  }'::jsonb
)
ON CONFLICT (id) DO NOTHING;

CREATE TABLE IF NOT EXISTS public.seabot_control_state (
  id integer PRIMARY KEY CHECK (id = 1),
  controller_id uuid REFERENCES public.seabot_accounts(id) ON DELETE SET NULL,
  controller_priority integer NOT NULL DEFAULT 10,
  controller_rank integer NOT NULL DEFAULT 0,
  command jsonb NOT NULL DEFAULT '{"type":"neutral","controlLock":{"status":"none"}}'::jsonb,
  version bigint NOT NULL DEFAULT 0,
  updated_at timestamptz NOT NULL DEFAULT now()
);

INSERT INTO public.seabot_control_state(id)
VALUES (1)
ON CONFLICT (id) DO NOTHING;

CREATE TABLE IF NOT EXISTS public.seabot_transparency_reports (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  title text NOT NULL,
  description text NOT NULL DEFAULT '',
  file_path text NOT NULL,
  file_name text NOT NULL DEFAULT '',
  mime_type text NOT NULL DEFAULT 'application/octet-stream',
  uploaded_by uuid REFERENCES public.seabot_accounts(id) ON DELETE SET NULL,
  created_at timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS public.seabot_poweroff_requests (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  requested_by uuid NOT NULL REFERENCES public.seabot_accounts(id) ON DELETE CASCADE,
  status text NOT NULL DEFAULT 'pending' CHECK (status IN ('pending','approved','denied','completed')),
  reviewed_by uuid REFERENCES public.seabot_accounts(id) ON DELETE SET NULL,
  response_message text NOT NULL DEFAULT '',
  created_at timestamptz NOT NULL DEFAULT now(),
  reviewed_at timestamptz,
  completed_at timestamptz
);
CREATE INDEX IF NOT EXISTS seabot_poweroff_requests_requested_idx
  ON public.seabot_poweroff_requests(requested_by, created_at DESC);
CREATE INDEX IF NOT EXISTS seabot_poweroff_requests_status_idx
  ON public.seabot_poweroff_requests(status, created_at DESC);

-- ---------------------------------------------------------------------------
-- AUTH -> SIBOAT PROFILE
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION public.seabot_priority_defaults(p_role text, p_is_lead boolean)
RETURNS TABLE(control_priority integer, visible_priority_rank integer, account_status text)
LANGUAGE sql
IMMUTABLE
SET search_path = public
AS $$
  SELECT
    CASE WHEN p_is_lead THEN 1000 WHEN p_role = 'developer' THEN 200 WHEN p_role = 'operator' THEN 100 ELSE 0 END,
    CASE WHEN p_is_lead THEN 1 WHEN p_role = 'developer' THEN 2 WHEN p_role = 'operator' THEN 5 ELSE 10 END,
    CASE WHEN p_is_lead OR p_role = 'visitor' THEN 'active' ELSE 'pending' END;
$$;

CREATE OR REPLACE FUNCTION public.seabot_handle_new_auth_user()
RETURNS trigger
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = public
AS $$
DECLARE
  v_name text := COALESCE(NULLIF(trim(new.raw_user_meta_data ->> 'name'), ''), NULLIF(trim(new.raw_user_meta_data ->> 'username'), ''), 'SIBOAT User');
  v_username text := COALESCE(NULLIF(trim(new.raw_user_meta_data ->> 'username'), ''), lower(split_part(COALESCE(new.email,''),'@',1)));
  v_requested_role text := lower(COALESCE(new.raw_user_meta_data ->> 'requested_role', 'visitor'));
  v_is_lead boolean := lower(COALESCE(v_username,'')) = lower('TRI-SEABOT.DEV')
                       AND lower(COALESCE(new.email,'')) = lower('triseabotdevteam@gmail.com');
  v_role text := CASE WHEN v_is_lead THEN 'developer' WHEN v_requested_role IN ('developer','operator') THEN v_requested_role ELSE 'visitor' END;
  v_status text := CASE WHEN v_is_lead OR v_role = 'visitor' THEN 'active' ELSE 'pending' END;
  v_priority integer := CASE WHEN v_is_lead THEN 1000 WHEN v_role='developer' THEN 200 WHEN v_role='operator' THEN 100 ELSE 0 END;
  v_rank integer := CASE WHEN v_is_lead THEN 1 WHEN v_role='developer' THEN 2 WHEN v_role='operator' THEN 5 ELSE 10 END;
BEGIN
  INSERT INTO public.seabot_accounts(id,name,username,email,role,status,is_lead,priority,priority_rank,comments,intro_seen)
  VALUES(new.id,v_name,v_username,lower(COALESCE(new.email,'')),v_role,v_status,v_is_lead,v_priority,v_rank,'',false)
  ON CONFLICT (id) DO UPDATE SET
    name=EXCLUDED.name,
    username=EXCLUDED.username,
    email=EXCLUDED.email,
    role=EXCLUDED.role,
    status=EXCLUDED.status,
    is_lead=EXCLUDED.is_lead,
    priority=EXCLUDED.priority,
    priority_rank=EXCLUDED.priority_rank;
  RETURN new;
END;
$$;

DROP TRIGGER IF EXISTS on_auth_user_created_seabot ON auth.users;
CREATE TRIGGER on_auth_user_created_seabot
AFTER INSERT ON auth.users
FOR EACH ROW EXECUTE FUNCTION public.seabot_handle_new_auth_user();

-- Keep the canonical Lead Developer identity stable when the Auth user exists.
DO $$
DECLARE v_id uuid;
BEGIN
  SELECT id INTO v_id FROM auth.users
  WHERE lower(email)=lower('triseabotdevteam@gmail.com')
  ORDER BY created_at ASC LIMIT 1;
  IF v_id IS NOT NULL THEN
    INSERT INTO public.seabot_accounts(id,name,username,email,role,status,is_lead,priority,priority_rank,intro_seen)
    VALUES(v_id,'SIBOAT Lead Developer','TRI-SEABOT.DEV',lower('triseabotdevteam@gmail.com'),'developer','active',true,1000,1,false)
    ON CONFLICT (id) DO UPDATE SET
      name=EXCLUDED.name, username=EXCLUDED.username, email=EXCLUDED.email,
      role='developer', status='active', is_lead=true, priority=1000, priority_rank=1;
  END IF;
END $$;

-- ---------------------------------------------------------------------------
-- AUTHORIZATION HELPERS
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION public.seabot_is_lead()
RETURNS boolean
LANGUAGE sql
STABLE
SECURITY DEFINER
SET search_path = public
AS $$
  SELECT EXISTS (
    SELECT 1 FROM public.seabot_accounts a
    WHERE a.id = auth.uid()
      AND a.status='active'
      AND (a.is_lead OR (lower(a.username)=lower('TRI-SEABOT.DEV') AND lower(a.email)=lower('triseabotdevteam@gmail.com') AND a.role='developer'))
  );
$$;

CREATE OR REPLACE FUNCTION public.seabot_my_account()
RETURNS public.seabot_accounts
LANGUAGE sql
STABLE
SECURITY DEFINER
SET search_path = public
AS $$ SELECT a FROM public.seabot_accounts a WHERE a.id=auth.uid() LIMIT 1; $$;

CREATE OR REPLACE FUNCTION public.seabot_auth_email(p_username text)
RETURNS text
LANGUAGE sql
STABLE
SECURITY DEFINER
SET search_path = public
AS $$
  SELECT a.email FROM public.seabot_accounts a
  WHERE lower(a.username)=lower(trim(p_username))
  LIMIT 1;
$$;

CREATE OR REPLACE FUNCTION public.seabot_rank(p_account_id uuid)
RETURNS integer
LANGUAGE sql
STABLE
SECURITY DEFINER
SET search_path = public
AS $$
  SELECT CASE WHEN a.is_lead THEN 3 WHEN a.role='developer' THEN 2 WHEN a.role='operator' THEN 1 ELSE 0 END
  FROM public.seabot_accounts a WHERE a.id=p_account_id;
$$;

CREATE OR REPLACE FUNCTION public.seabot_priority(p_account_id uuid)
RETURNS integer
LANGUAGE sql
STABLE
SECURITY DEFINER
SET search_path = public
AS $$ SELECT COALESCE((SELECT priority FROM public.seabot_accounts WHERE id=p_account_id),0); $$;

CREATE OR REPLACE FUNCTION public.seabot_can_control()
RETURNS boolean
LANGUAGE sql
STABLE
SECURITY DEFINER
SET search_path = public
AS $$
  SELECT EXISTS (
    SELECT 1
    FROM public.seabot_accounts a
    JOIN public.seabot_control_state c ON c.id=1
    WHERE a.id=auth.uid()
      AND a.status='active'
      AND (a.is_lead OR a.role IN ('developer','operator'))
      AND c.controller_id=auth.uid()
      AND COALESCE(c.command->'controlLock'->>'status','none') <> 'approved'
  );
$$;

CREATE OR REPLACE FUNCTION public.seabot_can_overwrite_control()
RETURNS boolean
LANGUAGE plpgsql
STABLE
SECURITY DEFINER
SET search_path = public
AS $$
DECLARE
  cur public.seabot_control_state;
  me public.seabot_accounts;
  me_score integer;
BEGIN
  SELECT * INTO me FROM public.seabot_accounts WHERE id=auth.uid();
  IF me.id IS NULL OR me.status <> 'active'
     OR NOT (me.is_lead OR me.role IN ('developer','operator')) THEN
    RETURN false;
  END IF;

  SELECT * INTO cur FROM public.seabot_control_state WHERE id=1;
  IF cur.controller_id IS NULL OR cur.controller_id=me.id THEN
    RETURN true;
  END IF;

  me_score := CASE
    WHEN me.is_lead THEN 1000
    WHEN me.priority >= 50 THEN me.priority
    WHEN me.role='developer' THEN 200
    WHEN me.role='operator' THEN 100
    ELSE 0
  END;

  RETURN me_score > COALESCE(cur.controller_priority,0)
      OR (me_score = COALESCE(cur.controller_priority,0)
          AND (CASE WHEN me.is_lead THEN 3 WHEN me.role='developer' THEN 2
                    WHEN me.role='operator' THEN 1 ELSE 0 END)
            > COALESCE(cur.controller_rank,0));
END;
$$;

CREATE OR REPLACE FUNCTION public.seabot_touch_presence()
RETURNS timestamptz
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = public
AS $$
DECLARE v_seen timestamptz;
BEGIN
  IF auth.uid() IS NULL THEN RAISE EXCEPTION USING errcode='42501',message='Authentication required.'; END IF;
  UPDATE public.seabot_accounts SET last_seen_at=now()
  WHERE id=auth.uid() AND (last_seen_at IS NULL OR last_seen_at < now()-interval '30 seconds')
  RETURNING last_seen_at INTO v_seen;
  IF v_seen IS NULL THEN SELECT last_seen_at INTO v_seen FROM public.seabot_accounts WHERE id=auth.uid(); END IF;
  RETURN v_seen;
END;
$$;

CREATE OR REPLACE FUNCTION public.seabot_presence_directory()
RETURNS TABLE(id uuid,name text,username text,role text,last_seen_at timestamptz)
LANGUAGE sql
STABLE
SECURITY DEFINER
SET search_path = public
AS $$ SELECT a.id,a.name,a.username,a.role,a.last_seen_at FROM public.seabot_accounts a WHERE a.status='active' ORDER BY a.priority ASC,a.created_at ASC; $$;

-- ---------------------------------------------------------------------------
-- LEAD ACCOUNT MANAGEMENT
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION public.seabot_guard_account_authorization_changes()
RETURNS trigger
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = public
AS $$
BEGIN
  /* Browser writes are protected by RLS plus this trigger. A NULL auth.uid()
     is allowed for trusted SQL-editor/server maintenance contexts. */
  IF auth.uid() IS NOT NULL AND NOT public.seabot_is_lead() THEN
    IF NEW.role IS DISTINCT FROM OLD.role
       OR NEW.priority IS DISTINCT FROM OLD.priority
       OR NEW.priority_rank IS DISTINCT FROM OLD.priority_rank
       OR NEW.is_lead IS DISTINCT FROM OLD.is_lead THEN
      RAISE EXCEPTION USING
        errcode='42501',
        message='Only the Lead Developer may change account access, priority, or Lead Developer state.';
    END IF;

    /* A signed-in user may voluntarily suspend their own account. */
    IF NEW.status IS DISTINCT FROM OLD.status
       AND NOT (NEW.id=auth.uid() AND NEW.status='suspended') THEN
      RAISE EXCEPTION USING
        errcode='42501',
        message='Only the Lead Developer may change another account status.';
    END IF;
  END IF;

  IF COALESCE(OLD.is_lead,false) AND NOT COALESCE(NEW.is_lead,false) THEN
    RAISE EXCEPTION USING errcode='42501',message='The Lead Developer account cannot be demoted.';
  END IF;
  IF COALESCE(NEW.is_lead,false) AND NEW.role <> 'developer' THEN
    RAISE EXCEPTION USING errcode='42501',message='A Lead Developer account must retain the developer role.';
  END IF;
  RETURN NEW;
END;
$$;
DROP TRIGGER IF EXISTS trg_seabot_guard_account_authorization_changes ON public.seabot_accounts;
CREATE TRIGGER trg_seabot_guard_account_authorization_changes BEFORE UPDATE ON public.seabot_accounts FOR EACH ROW EXECUTE FUNCTION public.seabot_guard_account_authorization_changes();

CREATE OR REPLACE FUNCTION public.seabot_lead_update_account(
  p_account_id uuid,p_role text DEFAULT NULL,p_status text DEFAULT NULL,p_priority integer DEFAULT NULL,
  p_priority_rank integer DEFAULT NULL,p_comments text DEFAULT NULL,p_name text DEFAULT NULL,p_username text DEFAULT NULL
)
RETURNS public.seabot_accounts
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = public
AS $$
DECLARE v_row public.seabot_accounts;
BEGIN
  IF NOT public.seabot_is_lead() THEN RAISE EXCEPTION USING errcode='42501',message='Lead Developer access is required.'; END IF;
  IF p_account_id IS NULL THEN RAISE EXCEPTION USING errcode='22023',message='Account ID is required.'; END IF;
  IF p_role IS NOT NULL AND p_role NOT IN ('developer','operator','visitor') THEN RAISE EXCEPTION USING errcode='22P02',message='Invalid account role.'; END IF;
  IF p_status IS NOT NULL AND p_status NOT IN ('active','pending','denied','suspended','blocked') THEN RAISE EXCEPTION USING errcode='22P02',message='Invalid account status.'; END IF;
  IF p_priority IS NOT NULL AND p_priority NOT BETWEEN 0 AND 1000 THEN RAISE EXCEPTION USING errcode='22023',message='Invalid control priority.'; END IF;
  IF p_priority_rank IS NOT NULL AND p_priority_rank NOT BETWEEN 1 AND 10 THEN RAISE EXCEPTION USING errcode='22023',message='Priority rank must be between 1 and 10.'; END IF;
  UPDATE public.seabot_accounts SET
    role=COALESCE(p_role,role),status=COALESCE(p_status,status),priority=COALESCE(p_priority,priority),
    priority_rank=COALESCE(p_priority_rank,priority_rank),comments=COALESCE(p_comments,comments),
    name=COALESCE(p_name,name),username=COALESCE(p_username,username)
  WHERE id=p_account_id RETURNING * INTO v_row;
  IF NOT FOUND THEN RAISE EXCEPTION USING errcode='P0002',message='Account not found.'; END IF;
  RETURN v_row;
END;
$$;

CREATE OR REPLACE FUNCTION public.seabot_lead_delete_account(p_account_id uuid)
RETURNS boolean
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = public
AS $$
DECLARE v_is_lead boolean;
BEGIN
  IF NOT public.seabot_is_lead() THEN RAISE EXCEPTION USING errcode='42501',message='Lead Developer access is required.'; END IF;
  SELECT COALESCE(is_lead,false) INTO v_is_lead FROM public.seabot_accounts WHERE id=p_account_id;
  IF NOT FOUND THEN RETURN false; END IF;
  IF v_is_lead THEN RAISE EXCEPTION USING errcode='42501',message='The Lead Developer account cannot be deleted.'; END IF;
  /* Supabase Storage owns its object metadata and should be mutated through
     the Storage API rather than by deleting rows directly. SIBOAT does not
     store ordinary user uploads, but fail safely if a future user owns one. */
  IF EXISTS (
    SELECT 1 FROM storage.objects
    WHERE owner_id = p_account_id::text
  ) THEN
    RAISE EXCEPTION USING
      errcode='55000',
      message='Account owns Supabase Storage objects. Remove or reassign those Storage objects before deleting this account.';
  END IF;

  /* Direct deletion of auth.users is intentional here: public.seabot_accounts
     references auth.users with ON DELETE CASCADE, so the Auth identity,
     sessions, and SIBOAT profile are removed together. */
  DELETE FROM auth.users WHERE id=p_account_id;
  RETURN FOUND;
END;
$$;

-- ---------------------------------------------------------------------------
-- POWER-OFF REQUEST RPCs
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION public.seabot_request_poweroff()
RETURNS public.seabot_poweroff_requests
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = public
AS $$
DECLARE v_row public.seabot_poweroff_requests;
BEGIN
  IF NOT EXISTS(SELECT 1 FROM public.seabot_accounts WHERE id=auth.uid() AND status='active' AND role IN ('developer','operator')) THEN RAISE EXCEPTION USING errcode='42501',message='Active Operator/Developer access is required.'; END IF;
  IF EXISTS(SELECT 1 FROM public.seabot_poweroff_requests WHERE requested_by=auth.uid() AND status='pending') THEN RAISE EXCEPTION USING errcode='23505',message='A power-off request is already pending.'; END IF;
  INSERT INTO public.seabot_poweroff_requests(requested_by) VALUES(auth.uid()) RETURNING * INTO v_row;
  RETURN v_row;
END;
$$;

CREATE OR REPLACE FUNCTION public.seabot_resolve_poweroff_request(p_request_id uuid,p_approve boolean)
RETURNS public.seabot_poweroff_requests
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = public
AS $$
DECLARE v_row public.seabot_poweroff_requests;
BEGIN
  IF NOT public.seabot_is_lead() THEN RAISE EXCEPTION USING errcode='42501',message='Lead Developer access is required.'; END IF;
  UPDATE public.seabot_poweroff_requests SET status=CASE WHEN p_approve THEN 'approved' ELSE 'denied' END,reviewed_by=auth.uid(),reviewed_at=now(),response_message=CASE WHEN p_approve THEN 'Approved by Lead Developer' ELSE 'Denied by Lead Developer' END
  WHERE id=p_request_id AND status='pending' RETURNING * INTO v_row;
  IF NOT FOUND THEN RAISE EXCEPTION USING errcode='P0002',message='Pending power-off request not found.'; END IF;
  RETURN v_row;
END;
$$;

CREATE OR REPLACE FUNCTION public.seabot_complete_poweroff_request(p_request_id uuid)
RETURNS boolean
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = public
AS $$
BEGIN
  IF NOT public.seabot_is_lead() THEN RAISE EXCEPTION USING errcode='42501',message='Lead Developer access is required.'; END IF;
  UPDATE public.seabot_poweroff_requests SET status='completed',completed_at=now() WHERE id=p_request_id AND status='approved';
  RETURN FOUND;
END;
$$;

-- ---------------------------------------------------------------------------
-- GLOBAL CONTROL LOCK PIN
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION public.seabot_unlock_control_with_pin(p_pin text)
RETURNS boolean
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = public
AS $$
DECLARE v_state public.seabot_control_state; v_command jsonb; v_lock jsonb;
BEGIN
  IF auth.uid() IS NULL THEN RAISE EXCEPTION USING errcode='42501',message='Authentication required.'; END IF;
  SELECT * INTO v_state FROM public.seabot_control_state WHERE id=1 FOR UPDATE;
  v_command:=COALESCE(v_state.command,'{}'::jsonb);v_lock:=COALESCE(v_command->'controlLock','{}'::jsonb);
  IF COALESCE(v_lock->>'status','none') <> 'approved' THEN RETURN false; END IF;
  IF COALESCE(v_lock->>'pin','') <> trim(p_pin) THEN RETURN false; END IF;
  v_command:=jsonb_set(v_command,'{controlLock}',jsonb_build_object('status','none','requestedAt',NULL,'approvedBy',NULL,'approvedAt',NULL,'pin',NULL),true);
  UPDATE public.seabot_control_state SET command=v_command,version=version+1,updated_at=now() WHERE id=1;
  RETURN true;
END;
$$;

-- ---------------------------------------------------------------------------
-- RLS / GRANTS
-- ---------------------------------------------------------------------------
GRANT USAGE ON SCHEMA public TO anon, authenticated;
GRANT SELECT ON public.seabot_accounts TO authenticated;
GRANT INSERT,UPDATE,DELETE ON public.seabot_accounts TO authenticated;
GRANT SELECT,INSERT,UPDATE ON public.seabot_access_requests TO authenticated;
GRANT SELECT,INSERT ON public.seabot_messages TO authenticated;
GRANT SELECT,INSERT,UPDATE ON public.seabot_app_settings TO authenticated;
GRANT SELECT,INSERT,UPDATE ON public.seabot_control_state TO authenticated;
GRANT SELECT,INSERT ON public.seabot_transparency_reports TO authenticated;
GRANT SELECT,INSERT,UPDATE ON public.seabot_poweroff_requests TO authenticated;

ALTER TABLE public.seabot_accounts ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.seabot_access_requests ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.seabot_messages ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.seabot_app_settings ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.seabot_control_state ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.seabot_transparency_reports ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.seabot_poweroff_requests ENABLE ROW LEVEL SECURITY;

DROP POLICY IF EXISTS seabot_accounts_select ON public.seabot_accounts;
CREATE POLICY seabot_accounts_select ON public.seabot_accounts FOR SELECT TO authenticated USING (true);
DROP POLICY IF EXISTS seabot_accounts_insert_self ON public.seabot_accounts;
CREATE POLICY seabot_accounts_insert_self ON public.seabot_accounts FOR INSERT TO authenticated WITH CHECK (id=auth.uid());
DROP POLICY IF EXISTS seabot_accounts_update_self_or_lead ON public.seabot_accounts;
CREATE POLICY seabot_accounts_update_self_or_lead ON public.seabot_accounts FOR UPDATE TO authenticated USING (id=auth.uid() OR public.seabot_is_lead()) WITH CHECK (id=auth.uid() OR public.seabot_is_lead());
DROP POLICY IF EXISTS seabot_accounts_delete_lead ON public.seabot_accounts;
CREATE POLICY seabot_accounts_delete_lead ON public.seabot_accounts FOR DELETE TO authenticated USING (public.seabot_is_lead() AND COALESCE(is_lead,false)=false);

DROP POLICY IF EXISTS seabot_access_select ON public.seabot_access_requests;
CREATE POLICY seabot_access_select ON public.seabot_access_requests FOR SELECT TO authenticated USING (account_id=auth.uid() OR public.seabot_is_lead());
DROP POLICY IF EXISTS seabot_access_insert ON public.seabot_access_requests;
CREATE POLICY seabot_access_insert ON public.seabot_access_requests FOR INSERT TO authenticated WITH CHECK (account_id=auth.uid());
DROP POLICY IF EXISTS seabot_access_update ON public.seabot_access_requests;
CREATE POLICY seabot_access_update ON public.seabot_access_requests FOR UPDATE TO authenticated USING (public.seabot_is_lead()) WITH CHECK (public.seabot_is_lead());

DROP POLICY IF EXISTS seabot_messages_select ON public.seabot_messages;
CREATE POLICY seabot_messages_select ON public.seabot_messages FOR SELECT TO authenticated USING (sender_id=auth.uid() OR recipient_id=auth.uid() OR public.seabot_is_lead());
DROP POLICY IF EXISTS seabot_messages_insert ON public.seabot_messages;
CREATE POLICY seabot_messages_insert ON public.seabot_messages FOR INSERT TO authenticated WITH CHECK (sender_id=auth.uid());

DROP POLICY IF EXISTS seabot_settings_select ON public.seabot_app_settings;
CREATE POLICY seabot_settings_select ON public.seabot_app_settings FOR SELECT TO authenticated USING (true);
DROP POLICY IF EXISTS seabot_settings_write ON public.seabot_app_settings;
CREATE POLICY seabot_settings_write ON public.seabot_app_settings FOR INSERT TO authenticated WITH CHECK (public.seabot_can_control());
DROP POLICY IF EXISTS seabot_settings_update ON public.seabot_app_settings;
CREATE POLICY seabot_settings_update ON public.seabot_app_settings FOR UPDATE TO authenticated USING (public.seabot_can_control()) WITH CHECK (public.seabot_can_control());

DROP POLICY IF EXISTS seabot_control_select ON public.seabot_control_state;
CREATE POLICY seabot_control_select ON public.seabot_control_state FOR SELECT TO authenticated USING (true);
DROP POLICY IF EXISTS seabot_control_write ON public.seabot_control_state;
CREATE POLICY seabot_control_write ON public.seabot_control_state
FOR INSERT TO authenticated
WITH CHECK (public.seabot_can_overwrite_control());

DROP POLICY IF EXISTS seabot_control_update ON public.seabot_control_state;
CREATE POLICY seabot_control_update ON public.seabot_control_state
FOR UPDATE TO authenticated
USING (public.seabot_can_overwrite_control())
WITH CHECK (public.seabot_can_overwrite_control());

DROP POLICY IF EXISTS seabot_reports_select ON public.seabot_transparency_reports;
CREATE POLICY seabot_reports_select ON public.seabot_transparency_reports FOR SELECT TO authenticated USING (true);
DROP POLICY IF EXISTS seabot_reports_insert ON public.seabot_transparency_reports;
CREATE POLICY seabot_reports_insert ON public.seabot_transparency_reports FOR INSERT TO authenticated WITH CHECK (public.seabot_is_lead() AND uploaded_by=auth.uid());

DROP POLICY IF EXISTS seabot_poweroff_select ON public.seabot_poweroff_requests;
CREATE POLICY seabot_poweroff_select ON public.seabot_poweroff_requests FOR SELECT TO authenticated USING (requested_by=auth.uid() OR public.seabot_is_lead());
DROP POLICY IF EXISTS seabot_poweroff_insert ON public.seabot_poweroff_requests;
CREATE POLICY seabot_poweroff_insert ON public.seabot_poweroff_requests FOR INSERT TO authenticated WITH CHECK (requested_by=auth.uid());
DROP POLICY IF EXISTS seabot_poweroff_update ON public.seabot_poweroff_requests;
CREATE POLICY seabot_poweroff_update ON public.seabot_poweroff_requests FOR UPDATE TO authenticated USING (public.seabot_is_lead()) WITH CHECK (public.seabot_is_lead());

-- Function privileges: expose only the calls used by the browser.
REVOKE ALL ON FUNCTION public.seabot_auth_email(text) FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_auth_email(text) TO anon,authenticated;
REVOKE ALL ON FUNCTION public.seabot_my_account() FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_my_account() TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_is_lead() FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_is_lead() TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_rank(uuid) FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_rank(uuid) TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_priority(uuid) FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_priority(uuid) TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_can_control() FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_can_control() TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_can_overwrite_control() FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_can_overwrite_control() TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_touch_presence() FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_touch_presence() TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_presence_directory() FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_presence_directory() TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_lead_update_account(uuid,text,text,integer,integer,text,text,text) FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_lead_update_account(uuid,text,text,integer,integer,text,text,text) TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_lead_delete_account(uuid) FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_lead_delete_account(uuid) TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_request_poweroff() FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_request_poweroff() TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_resolve_poweroff_request(uuid,boolean) FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_resolve_poweroff_request(uuid,boolean) TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_complete_poweroff_request(uuid) FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_complete_poweroff_request(uuid) TO authenticated;
REVOKE ALL ON FUNCTION public.seabot_unlock_control_with_pin(text) FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_unlock_control_with_pin(text) TO authenticated;

-- ---------------------------------------------------------------------------
-- REALTIME
-- ---------------------------------------------------------------------------
ALTER TABLE public.seabot_accounts REPLICA IDENTITY FULL;
ALTER TABLE public.seabot_access_requests REPLICA IDENTITY FULL;
ALTER TABLE public.seabot_messages REPLICA IDENTITY FULL;
ALTER TABLE public.seabot_app_settings REPLICA IDENTITY FULL;
ALTER TABLE public.seabot_control_state REPLICA IDENTITY FULL;
ALTER TABLE public.seabot_poweroff_requests REPLICA IDENTITY FULL;

DO $$
DECLARE t text;
BEGIN
  IF EXISTS(SELECT 1 FROM pg_publication WHERE pubname='supabase_realtime') THEN
    FOREACH t IN ARRAY ARRAY['seabot_accounts','seabot_access_requests','seabot_messages','seabot_app_settings','seabot_control_state','seabot_poweroff_requests'] LOOP
      IF to_regclass('public.'||t) IS NOT NULL AND NOT EXISTS(SELECT 1 FROM pg_publication_tables WHERE pubname='supabase_realtime' AND schemaname='public' AND tablename=t) THEN
        EXECUTE format('ALTER PUBLICATION supabase_realtime ADD TABLE public.%I',t);
      END IF;
    END LOOP;
  END IF;
END $$;

-- ---------------------------------------------------------------------------
-- STORAGE: public transparency bucket
-- ---------------------------------------------------------------------------
INSERT INTO storage.buckets(id,name,public)
VALUES('transparency','transparency',true)
ON CONFLICT(id) DO UPDATE SET public=true;

DROP POLICY IF EXISTS seabot_transparency_storage_select ON storage.objects;
CREATE POLICY seabot_transparency_storage_select ON storage.objects FOR SELECT TO public USING (bucket_id='transparency');
DROP POLICY IF EXISTS seabot_transparency_storage_insert ON storage.objects;
CREATE POLICY seabot_transparency_storage_insert ON storage.objects FOR INSERT TO authenticated WITH CHECK (bucket_id='transparency' AND public.seabot_is_lead());
DROP POLICY IF EXISTS seabot_transparency_storage_delete ON storage.objects;
CREATE POLICY seabot_transparency_storage_delete ON storage.objects FOR DELETE TO authenticated USING (bucket_id='transparency' AND public.seabot_is_lead());

DO $$
DECLARE missing text[] := ARRAY[]::text[];
BEGIN
  IF to_regclass('public.seabot_accounts') IS NULL THEN missing := array_append(missing,'public.seabot_accounts'); END IF;
  IF to_regclass('public.seabot_access_requests') IS NULL THEN missing := array_append(missing,'public.seabot_access_requests'); END IF;
  IF to_regclass('public.seabot_messages') IS NULL THEN missing := array_append(missing,'public.seabot_messages'); END IF;
  IF to_regclass('public.seabot_app_settings') IS NULL THEN missing := array_append(missing,'public.seabot_app_settings'); END IF;
  IF to_regclass('public.seabot_control_state') IS NULL THEN missing := array_append(missing,'public.seabot_control_state'); END IF;
  IF to_regclass('public.seabot_transparency_reports') IS NULL THEN missing := array_append(missing,'public.seabot_transparency_reports'); END IF;
  IF to_regclass('public.seabot_poweroff_requests') IS NULL THEN missing := array_append(missing,'public.seabot_poweroff_requests'); END IF;
  IF NOT EXISTS(SELECT 1 FROM pg_proc p JOIN pg_namespace n ON n.oid=p.pronamespace WHERE n.nspname='public' AND p.proname='seabot_lead_delete_account' AND p.pronargs=1) THEN missing := array_append(missing,'public.seabot_lead_delete_account(uuid)'); END IF;
  IF NOT EXISTS(SELECT 1 FROM pg_proc p JOIN pg_namespace n ON n.oid=p.pronamespace WHERE n.nspname='public' AND p.proname='seabot_lead_update_account' AND p.pronargs=8) THEN missing := array_append(missing,'public.seabot_lead_update_account(uuid,text,text,integer,integer,text,text,text)'); END IF;
  IF cardinality(missing)>0 THEN
    RAISE EXCEPTION USING errcode='42883', message='SIBOAT setup preflight failed: '||array_to_string(missing,', ');
  END IF;
END $$;

COMMIT;

-- POST-SETUP CHECKS (run separately if desired):
-- SELECT id,name,username,role,status,is_lead,priority,priority_rank FROM public.seabot_accounts ORDER BY priority ASC;
-- SELECT proname,pg_get_function_identity_arguments(oid) FROM pg_proc WHERE pronamespace='public'::regnamespace AND proname LIKE 'seabot_%' ORDER BY proname;
-- SELECT * FROM public.seabot_control_state WHERE id=1;
--
-- Hosted Supabase Auth setting still required:
-- Authentication -> Providers -> Email -> Confirm email = OFF.

-- ---------------------------------------------------------------------------
-- ACCOUNT DELETION BEHAVIOR
-- ---------------------------------------------------------------------------
-- The Lead Developer delete RPC removes the Auth user and the public profile
-- cascades from auth.users. Existing JWT access tokens are stateless and can
-- remain cryptographically valid until their exp time; however, the deleted
-- account row no longer satisfies SIBOAT RLS checks, so application data
-- access is denied immediately. Supabase Auth refresh/session records are
-- removed with the Auth user.


-- ================================================================
-- SIBOAT ANALYTICS ADD-ON / DETECTION HISTORY
-- ================================================================
-- SIBOAT DATABASE ANALYTICS ADD-ON
-- Apply AFTER SIBOAT_NEW_DATABASE_2026-09-17.sql
-- Adds durable detection-event history used by the SIBOAT dashboard.
-- No service_role or secret key belongs in the dashboard HTML.

BEGIN;

CREATE TABLE IF NOT EXISTS public.seabot_detection_events (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  event_seq bigint NOT NULL,
  event_type text NOT NULL DEFAULT 'TELEMETRY',
  category text NOT NULL CHECK (category IN ('CAT1','CAT2','CAT3','REJECT','UNKNOWN')),
  outcome text NOT NULL DEFAULT '' CHECK (outcome IN ('accepted','rejected','timeout','error','')),
  sub_label text NOT NULL DEFAULT '',
  confidence real NOT NULL DEFAULT 0 CHECK (confidence >= 0 AND confidence <= 1),
  distance_cm real,
  servo_used integer CHECK (servo_used IN (0,1,2)),
  main_ip inet,
  camera_ip inet,
  stationary_mode boolean NOT NULL DEFAULT false,
  metadata jsonb NOT NULL DEFAULT '{}'::jsonb,
  recorded_by uuid REFERENCES public.seabot_accounts(id) ON DELETE SET NULL,
  created_at timestamptz NOT NULL DEFAULT now()
);

CREATE UNIQUE INDEX IF NOT EXISTS seabot_detection_events_seq_uidx
  ON public.seabot_detection_events(event_seq);
CREATE INDEX IF NOT EXISTS seabot_detection_events_created_idx
  ON public.seabot_detection_events(created_at DESC);
CREATE INDEX IF NOT EXISTS seabot_detection_events_category_idx
  ON public.seabot_detection_events(category, created_at DESC);
CREATE INDEX IF NOT EXISTS seabot_detection_events_label_idx
  ON public.seabot_detection_events(sub_label, created_at DESC);

ALTER TABLE public.seabot_detection_events ENABLE ROW LEVEL SECURITY;

DROP POLICY IF EXISTS seabot_detection_select ON public.seabot_detection_events;
CREATE POLICY seabot_detection_select
  ON public.seabot_detection_events
  FOR SELECT TO authenticated
  USING (
    EXISTS (
      SELECT 1 FROM public.seabot_accounts a
      WHERE a.id=auth.uid()
        AND a.status='active'
        AND (a.is_lead OR a.role IN ('developer','operator'))
    )
  );

DROP POLICY IF EXISTS seabot_detection_insert ON public.seabot_detection_events;
CREATE POLICY seabot_detection_insert
  ON public.seabot_detection_events
  FOR INSERT TO authenticated
  WITH CHECK (
    recorded_by=auth.uid()
    AND EXISTS (
      SELECT 1 FROM public.seabot_accounts a
      WHERE a.id=auth.uid()
        AND a.status='active'
        AND (a.is_lead OR a.role IN ('developer','operator'))
    )
  );

GRANT SELECT, INSERT ON public.seabot_detection_events TO authenticated;

-- The ESP32 remains the local source of truth while the dashboard/browser is connected.
-- The dashboard caches event history locally and queues Supabase inserts while offline.

COMMIT;

-- Optional verification:
-- SELECT event_type,category,sub_label,confidence,outcome,created_at
-- FROM public.seabot_detection_events ORDER BY created_at DESC LIMIT 50;


-- ================================================================
-- SIBOAT LOCAL DEVICE DISCOVERY
-- ================================================================
-- The ESP32 remains a local device. This registry is only a rendezvous
-- service for the hosted Web/Android dashboard to learn the ESP32's current
-- LAN address. The dashboard STILL verifies the discovered address by calling
-- the device's local /api/status endpoint before accepting it.

BEGIN;

CREATE TABLE IF NOT EXISTS public.seabot_device_registry (
  device_id text PRIMARY KEY,
  device_type text NOT NULL CHECK (device_type IN ('main','cam')),
  local_ip text NOT NULL,
  http_port integer NOT NULL DEFAULT 80 CHECK (http_port BETWEEN 1 AND 65535),
  ws_port integer NOT NULL DEFAULT 81 CHECK (ws_port BETWEEN 1 AND 65535),
  hostname text NOT NULL DEFAULT '',
  firmware_version text NOT NULL DEFAULT '',
  ssid text NOT NULL DEFAULT '',
  mac text NOT NULL DEFAULT '',
  boot_id text NOT NULL DEFAULT '',
  camera_ip text NOT NULL DEFAULT '',
  last_seen_at timestamptz NOT NULL DEFAULT now(),
  updated_at timestamptz NOT NULL DEFAULT now()
);

ALTER TABLE public.seabot_device_registry ADD COLUMN IF NOT EXISTS camera_ip text NOT NULL DEFAULT '';

CREATE INDEX IF NOT EXISTS seabot_device_registry_last_seen_idx
  ON public.seabot_device_registry(last_seen_at DESC);

ALTER TABLE public.seabot_device_registry ENABLE ROW LEVEL SECURITY;

DROP POLICY IF EXISTS seabot_device_registry_select ON public.seabot_device_registry;
CREATE POLICY seabot_device_registry_select
  ON public.seabot_device_registry
  FOR SELECT TO authenticated
  USING (true);

REVOKE ALL ON TABLE public.seabot_device_registry FROM PUBLIC;
GRANT SELECT ON TABLE public.seabot_device_registry TO authenticated;

DROP FUNCTION IF EXISTS public.seabot_register_device(text,text,text,integer,integer,text,text,text,text);

CREATE OR REPLACE FUNCTION public.seabot_register_device(
  p_device_id text,
  p_device_type text,
  p_local_ip text,
  p_http_port integer DEFAULT 80,
  p_ws_port integer DEFAULT 81,
  p_firmware_version text DEFAULT '',
  p_ssid text DEFAULT '',
  p_mac text DEFAULT '',
  p_boot_id text DEFAULT '',
  p_camera_ip text DEFAULT ''
)
RETURNS public.seabot_device_registry
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = public
AS $$
DECLARE
  v_row public.seabot_device_registry;
BEGIN
  IF p_device_id NOT IN ('SIBOAT-MAIN','SIBOAT-CAM') THEN
    RAISE EXCEPTION USING errcode='22023', message='Unknown SIBOAT device id.';
  END IF;

  IF p_device_type NOT IN ('main','cam') THEN
    RAISE EXCEPTION USING errcode='22023', message='Invalid SIBOAT device type.';
  END IF;

  IF p_local_ip !~ '^([0-9]{1,3}\.){3}[0-9]{1,3}$' THEN
    RAISE EXCEPTION USING errcode='22023', message='Invalid IPv4 address.';
  END IF;

  IF split_part(p_local_ip,'.',1)::integer > 255
     OR split_part(p_local_ip,'.',2)::integer > 255
     OR split_part(p_local_ip,'.',3)::integer > 255
     OR split_part(p_local_ip,'.',4)::integer > 255 THEN
    RAISE EXCEPTION USING errcode='22023', message='Invalid IPv4 address.';
  END IF;

  INSERT INTO public.seabot_device_registry(
    device_id,device_type,local_ip,http_port,ws_port,hostname,
    firmware_version,ssid,mac,boot_id,camera_ip,last_seen_at,updated_at
  )
  VALUES(
    p_device_id,p_device_type,p_local_ip,p_http_port,p_ws_port,
    CASE WHEN p_device_id='SIBOAT-MAIN' THEN 'siboat.local' ELSE 'siboat-cam.local' END,
    COALESCE(p_firmware_version,''),COALESCE(p_ssid,''),COALESCE(p_mac,''),
    COALESCE(p_boot_id,''),COALESCE(p_camera_ip,''),now(),now()
  )
  ON CONFLICT(device_id) DO UPDATE SET
    device_type=EXCLUDED.device_type,
    local_ip=EXCLUDED.local_ip,
    http_port=EXCLUDED.http_port,
    ws_port=EXCLUDED.ws_port,
    hostname=EXCLUDED.hostname,
    firmware_version=EXCLUDED.firmware_version,
    ssid=EXCLUDED.ssid,
    mac=EXCLUDED.mac,
    boot_id=EXCLUDED.boot_id,
    camera_ip=EXCLUDED.camera_ip,
    last_seen_at=now(),
    updated_at=now()
  RETURNING * INTO v_row;

  RETURN v_row;
END;
$$;

REVOKE ALL ON FUNCTION public.seabot_register_device(text,text,text,integer,integer,text,text,text,text,text) FROM PUBLIC;
GRANT EXECUTE ON FUNCTION public.seabot_register_device(text,text,text,integer,integer,text,text,text,text,text) TO anon,authenticated;

COMMIT;

-- Optional verification:
-- SELECT device_id,device_type,local_ip,http_port,ws_port,firmware_version,last_seen_at
-- FROM public.seabot_device_registry ORDER BY device_id;

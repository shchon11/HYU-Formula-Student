# Labeling from off the network

The Jetson sits on campus WiFi at a private address behind NAT, so there is no
port to forward and nothing outside can dial in. The way around that is a tunnel
the Jetson opens outbound: Cloudflare hands back a public https URL and forwards
requests down the connection the Jetson already holds open.

    src/perception/scripts/serve_labeler.sh datasets/0801_cones

That starts `cone_labeler_web.py` on `127.0.0.1:8770` and a Cloudflare quick
tunnel in front of it, then prints the URL to hand out. Ctrl+C stops both.

## What is exposed

- The web labeler and nothing else. The server binds to loopback, so even on the
  campus network there is no open port; every request arrives through the tunnel.
- Requests must carry an access token. It lives in `<dataset>/.labeler_token`
  (mode 600) and is reused across restarts so the links keep working. The first
  visit trades `?t=<token>` for a cookie. No token, no images, no API.
- The token is a shared secret, not per-person accounts. Anyone holding the URL
  can label and can delete frames (into `_trash/`, recoverable). Share it in a
  team channel, not anywhere public, and rotate it by deleting
  `<dataset>/.labeler_token` and restarting.

A quick tunnel gets a new random hostname on every restart. That is fine for a
weekend of labeling; for something stable, or for real per-person logins, use a
named tunnel.

## Named tunnel with Cloudflare Access

Needs a Cloudflare account and a domain on it. One-time setup:

    cloudflared tunnel login                       # browser auth, writes a cert
    cloudflared tunnel create fsk-labeler
    cloudflared tunnel route dns fsk-labeler labeler.example.com

Then run against it:

    src/perception/scripts/serve_labeler.sh datasets/0801_cones --tunnel fsk-labeler

`https://labeler.example.com/?t=<token>` is now stable. In the Cloudflare Zero
Trust dashboard add an Access application for that hostname and a policy that
allows your teammates' emails (Google or GitHub login, or a one-time PIN mailed
to them). Access authenticates at Cloudflare's edge, so unauthenticated requests
never reach the Jetson at all, and the app token stays as a second layer.

## Handing it to the team

The printed URL prompts for a name on first load. A per-person bookmark skips
that prompt:

    https://<host>/?t=<token>&user=sojun

The name is only an attribution label -- it is recorded next to every frame that
person marks reviewed, and it is what `stats` breaks down by. It is not a login.

## Working in parallel

Everyone shares one copy of the dataset on the Jetson; there is nothing to merge
afterwards.

- **next unreviewed** (`N`) claims a frame. A frame someone opens is held in
  their name for three minutes, and claims skip frames another person holds, so
  two people are never handed the same one.
- The frame list shows a yellow name next to any frame someone else is on.
- Progress from everyone else appears within about twelve seconds.
- Saves are per frame and last-write-wins. Two people editing the same frame at
  the same time is the one case that loses work, which is what the hold system
  exists to prevent -- so use `N` rather than scrolling to a random frame.

## Same dataset, both tools

`cone_labeler.py` (the PyQt app on the Jetson) and `cone_labeler_web.py` read and
write the same `images/`, `labels/` and `.labeler_state.json`. A frame reviewed
in one counts as reviewed in the other, and `remap_labels.py` keeps skipping
reviewed frames. Running both at once is safe as long as they are not on the same
frame; the desktop tool does not participate in the hold system.

## If something goes wrong

- `cloudflared exited` -- usually no outbound internet. Check with
  `curl -I https://api.cloudflare.com`.
- 403 in the browser -- the cookie expired or the token was rotated. Open the
  `?t=<token>` URL again.
- 502/530 on the public URL -- the tunnel is up but the local server died. Look
  at the server log path the script prints.
- Slow images -- every frame is a full 1280x720 JPEG through the tunnel. It is
  the campus uplink, not the Jetson.

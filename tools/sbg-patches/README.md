# Humble compatibility patch for the vendored SBG driver

`external.repos` pins `sbg_ros2_driver` to **3.3.2**, but that tag targets
Iron+ and does not compile on **Humble**:

```
fatal error: tf2_ros/transform_broadcaster.hpp: No such file or directory
```

sbg_driver 3.3.1 replaced two tf2_ros C headers (`.h`) with the `.hpp` variant
(changelog: *"removed usage of deprecated tf2 C header"*). Those `.hpp` shims
exist on Iron+ only. Humble ships just:

- `tf2_ros/transform_broadcaster.h`
- `tf2_ros/static_transform_broadcaster.h`

(`tf2/LinearMath/Quaternion.hpp` and `tf2_geometry_msgs/tf2_geometry_msgs.hpp`
**do** exist on Humble, so they are left untouched.)

`patch-sbg-humble.sh` reverts only those two includes back to `.h`. It is
idempotent — re-running on an already-patched (or already-`.h`) tree is a no-op.

## Usage

The driver now lives **directly in this repo** (`sbg_ros2_driver/`, MIT-licensed)
with these patches already applied and committed — it is no longer vcs-imported,
so nothing has to be re-applied for a normal build. This script is kept for one
job: **refreshing from upstream**. Clone a new upstream tag into a scratch dir,
run the script against it, and diff the result into `sbg_ros2_driver/`:

```bash
git clone -b 3.3.2 https://github.com/SBG-Systems/sbg_ros2_driver.git /tmp/sbg-new
SBG_SRC=/tmp/sbg-new bash src/tools/sbg-patches/patch-sbg-humble.sh
# then diff /tmp/sbg-new into src/sbg_ros2_driver/ and keep our local additions
# (scripts/ntrip_client.py, the config output/rtcm edits, launch wiring)
```

What the script applies: the Humble tf2 `.hpp`->`.h` include fix, and the config
edits `log_ekf_rot_accel_body: 8` and `rtcm.subscribe: true`.

## Alternative

If you'd rather run a stock, unpatched tree, pin `external.repos` to **3.3.0**
— the last tag that still uses the `.h` includes — instead of 3.3.2. You lose
the 3.3.1/3.3.2 changes (deprecation cleanup + updated sbgECom v5.3.2276).

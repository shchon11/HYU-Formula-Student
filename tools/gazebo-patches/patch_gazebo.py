#!/usr/bin/env python3
"""Apply the FSK GPU-laser patches to a gazebo-classic source tree.

Usage: python3 patch_gazebo.py <gazebo-classic-src-root>

Three patches, verified against tag gazebo11_11.10.2 (the exact version Ubuntu
22.04 / ROS Humble ships). All three preserve stock behavior unless the env
knob is set; re-running on a patched tree is a no-op.

1. gazebo/sensors/GpuRaySensor.cc
   The first-pass depth texture has a hardcoded 2048-texel floor. Make it
   tunable via GAZEBO_GPU_LASER_TEX_MIN (default 2048 = stock, clamped to
   [16, 16384]) and gzmsg the chosen geometry once per sensor so a patched
   gzserver is recognizable at runtime. The vertical texture height follows
   automatically through the existing aspect-ratio branch, so raising the
   floor shrinks BOTH the azimuth and the elevation texel quantization while
   the published scan keeps its exact sample counts (a real VLP-16 stays a
   real VLP-16).

2. media/materials/scripts/gazebo.material
   The first-pass laser material culled backfaces. A lidar return does not
   depend on triangle winding; with culling on, meshes with flipped normals
   silently vanish from the scan while looking fine on camera.

3. media/materials/programs/laser_2nd_pass.frag
   Beams that map exactly onto a camera-face boundary can land outside [0,1]
   texcoords by a float ulp; the stock shader paints them white, and because
   the first pass stores METRIC range in the R channel, white decodes as a
   phantom return at exactly 1.0 m. Clamp to the border texel instead.

4. gazebo/rendering/GpuLaser.cc (RenderImpl)
   SkyX's dome and volumetric clouds are REAL scene geometry, so the laser
   depth pass returns them like a solid ceiling: measured on skidpad_kase2026,
   every sky-bound beam (12,027 of 28,800 per scan) came back as a phantom
   point at 17-21 m, and far cones sitting at that range got occluded. A lidar
   gets no return from sky; hide SkyX while the laser renders. Upstream
   already does this permanently for the moon node ("it clips gpu laser range
   values") but left the dome and clouds visible.
"""
import re
import sys
from pathlib import Path

NEW_FRAG = """uniform sampler2D tex1;
uniform sampler2D tex2;
uniform sampler2D tex3;

uniform vec4 texSize;
varying float tex;

// FSK patch: the first pass stores METRIC range in R. The stock shader
// painted out-of-[0,1] texcoords white, so a beam that lands on a
// camera-face boundary and falls outside by a float ulp became a phantom
// return at exactly 1.0 m. The correct sample for such a beam is the
// border texel: clamp instead of painting white.
void main()
{
  vec2 uv = clamp(gl_TexCoord[0].st, 0.0, 1.0);
  int int_tex = int(tex * 1000.0);
  if (int_tex == 0)
    gl_FragColor = texture2D(tex1, uv);
  else if (int_tex == 1)
    gl_FragColor = texture2D(tex2, uv);
  else
    gl_FragColor = texture2D(tex3, uv);
}
"""


def die(msg):
    sys.exit(f"patch_gazebo: FAILED: {msg}")


def sub_once(text, pattern, repl, what, path):
    new, n = re.subn(pattern, repl, text, count=1)
    if n != 1:
        die(f"pattern for '{what}' not found in {path} "
            f"(upstream drift? this patcher expects tag gazebo11_11.10.2)")
    return new


def patch_sensor_cc(root):
    cc = root / "gazebo" / "sensors" / "GpuRaySensor.cc"
    if not cc.is_file():
        die(f"{cc} missing")
    t = cc.read_text()
    if "GAZEBO_GPU_LASER_TEX_MIN" in t:
        print(f"skip (already patched): {cc}")
        return

    t = sub_once(t, re.escape("#include <functional>"),
                 "#include <cstdlib>\n#include <functional>",
                 "cstdlib include", cc)

    old_block = (r"unsigned int horzRangeCountPerCamera =\s*\n\s*"
                 r"std::max\(2048U, this->dataPtr->horzRangeCount / cameraCount\);")
    new_block = (
        "// FSK patch: the fixed 2048 texture floor bounds texel quantization\n"
        "    // horizontally; expose it so an idle GPU can buy back accuracy. The\n"
        "    // vertical texture height follows through the aspect-ratio branch\n"
        "    // below, so one knob tightens both axes.\n"
        "    unsigned int texMin = 2048u;\n"
        "    if (const char *texMinEnv = std::getenv(\"GAZEBO_GPU_LASER_TEX_MIN\"))\n"
        "    {\n"
        "      texMin = static_cast<unsigned int>(\n"
        "          ignition::math::clamp(std::atoi(texMinEnv), 16, 16384));\n"
        "    }\n"
        "    unsigned int horzRangeCountPerCamera =\n"
        "        std::max(texMin, this->dataPtr->horzRangeCount / cameraCount);")
    t = sub_once(t, old_block, new_block, "texture floor block", cc)

    anchor = r"\n(\s*)// Initialize camera sdf for GpuLaser"
    gzmsg = (
        "\n\\1gzmsg << \"GpuRaySensor [\" << this->Name() << \"]: cameras=\"\n"
        "\\1      << cameraCount << \" first-pass tex=\" << horzRangeCountPerCamera\n"
        "\\1      << \"x\" << vertRangeCountPerCamera\n"
        "\\1      << \" (GAZEBO_GPU_LASER_TEX_MIN=\" << texMin << \")\" << std::endl;\n"
        "\n\\1// Initialize camera sdf for GpuLaser")
    t = sub_once(t, anchor, gzmsg, "gzmsg anchor", cc)

    cc.write_text(t)
    print(f"patched: {cc}")


def patch_material(root):
    mat = root / "media" / "materials" / "scripts" / "gazebo.material"
    if not mat.is_file():
        die(f"{mat} missing")
    t = mat.read_text()
    if re.search(r"LaserScan1st[\s\S]{0,500}cull_hardware", t):
        print(f"skip (already patched): {mat}")
        return
    pat = (r"(material Gazebo/LaserScan1st\s*\{\s*technique\s*\{\s*"
           r"pass laser_tex\s*\{\s*separate_scene_blend one zero one zero)")
    repl = ("\\1\n\n"
            "      // FSK patch: a lidar return does not depend on triangle winding;\n"
            "      // with culling on, meshes with flipped normals vanish from the\n"
            "      // scan while looking fine on camera.\n"
            "      cull_hardware none\n"
            "      cull_software none")
    t = sub_once(t, pat, repl, "LaserScan1st cull", mat)
    mat.write_text(t)
    print(f"patched: {mat}")


def patch_frag(root):
    frag = root / "media" / "materials" / "programs" / "laser_2nd_pass.frag"
    if not frag.is_file():
        die(f"{frag} missing")
    t = frag.read_text()
    if "clamp(" in t:
        print(f"skip (already patched): {frag}")
        return
    if "vec4(1,1,1,1)" not in t.replace(" ", ""):
        die(f"{frag} does not look like the stock shader; refusing to overwrite")
    frag.write_text(NEW_FRAG)
    print(f"patched: {frag}")


def patch_gpulaser_sky(root):
    cc = root / "gazebo" / "rendering" / "GpuLaser.cc"
    if not cc.is_file():
        die(f"{cc} missing")
    t = cc.read_text()
    if "GetSkyX()" in t:
        print(f"skip (already patched): {cc}")
        return

    t = sub_once(t, re.escape('#include "gazebo/rendering/Scene.hh"'),
                 '#include "gazebo/rendering/Scene.hh"\n'
                 '#include "gazebo/rendering/skyx/include/SkyX.h"',
                 "SkyX include", cc)

    old1 = re.escape(
        "Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();\n\n"
        "  sceneMgr->_suppressRenderStateChanges(true);")
    new1 = (
        "Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();\n\n"
        "  // FSK patch: SkyX's dome and volumetric clouds are real scene geometry,\n"
        "  // so the laser depth pass returns them like a solid ceiling (~20 m\n"
        "  // overhead): every sky-bound beam becomes a phantom point and far cones\n"
        "  // get occluded. A lidar gets no return from sky; hide SkyX while the\n"
        "  // laser renders (upstream already does this permanently for the moon).\n"
        "  SkyX::SkyX *skyx = this->scene->GetSkyX();\n"
        "  const bool skyxVisible = skyx && skyx->isVisible();\n"
        "  if (skyxVisible)\n"
        "    skyx->setVisible(false);\n\n"
        "  sceneMgr->_suppressRenderStateChanges(true);")
    t = sub_once(t, old1, new1, "SkyX hide before first pass", cc)

    old2 = re.escape(
        "sceneMgr->_suppressRenderStateChanges(false);\n\n"
        "  double secondPassDur")
    new2 = (
        "sceneMgr->_suppressRenderStateChanges(false);\n\n"
        "  if (skyxVisible)\n"
        "    skyx->setVisible(true);\n\n"
        "  double secondPassDur")
    t = sub_once(t, old2, new2, "SkyX restore after second pass", cc)

    cc.write_text(t)
    print(f"patched: {cc}")


def main():
    if len(sys.argv) != 2:
        die("usage: patch_gazebo.py <gazebo-classic-src-root>")
    root = Path(sys.argv[1]).resolve()
    if not (root / "gazebo").is_dir():
        die(f"{root} is not a gazebo-classic source tree")
    patch_sensor_cc(root)
    patch_material(root)
    patch_frag(root)
    patch_gpulaser_sky(root)
    print("patch_gazebo: all patches applied.")


if __name__ == "__main__":
    main()

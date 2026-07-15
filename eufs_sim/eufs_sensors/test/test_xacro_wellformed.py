"""Every xacro in this package must be well-formed XML.

A `--` inside an XML comment is not a comment, it is a parse error, and xacro
reports it as "line N, column M" of a file it does not name. That signature has
cost this repo several debugging sessions: the sim just fails to launch and the
message points at whichever include happens to be first. Catching it at build
time turns a launch-time scavenger hunt into a named failing test.
"""
import glob
import os
import xml.dom.minidom

import pytest

URDF_DIR = os.path.join(os.path.dirname(__file__), "..", "urdf")
XACROS = sorted(glob.glob(os.path.join(URDF_DIR, "*.xacro")))


def test_found_some_xacros():
    assert XACROS, f"no xacro files under {URDF_DIR}"


@pytest.mark.parametrize("path", XACROS,
                         ids=[os.path.basename(p) for p in XACROS])
def test_wellformed(path):
    try:
        xml.dom.minidom.parse(path)
    except Exception as exc:                                  # noqa: BLE001
        pytest.fail(f"{os.path.basename(path)} is not well-formed XML: {exc}\n"
                    f"  (a '--' inside an XML comment is the usual cause)")

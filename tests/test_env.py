import json

from jointbench.cli import main
from jointbench.env import tool_vs_header


def test_env_writes_a_record_without_personal_details(tmp_path, monkeypatch):
    monkeypatch.setenv("JOINTBENCH_BOX_IMAGE", "stepbench/box-model:v1")
    monkeypatch.setenv("JOINTBENCH_BOX_DIGEST", "sha256:abc")
    assert main(["env", str(tmp_path), "--model", "m-1", "--provider", "openrouter", "--tool", "build123d",
                 "--mcp", "freecad-mcp=0.4.1", "--no-programs"]) == 0
    text = (tmp_path / "environment.json").read_text()
    rec = json.loads(text)
    g, m = rec["given_by_harness"], rec["measured"]
    assert g["model"] == "m-1" and g["provider"] == "openrouter" and g["mcp_servers"] == {"freecad-mcp": "0.4.1"}
    assert "build123d" in m["python_packages"]
    assert m["machine"]["box"] == {"image": "stepbench/box-model:v1", "digest": "sha256:abc"}
    import os
    import socket

    assert socket.gethostname() not in text
    assert os.path.expanduser("~") not in text


def test_declared_tool_against_step_header():
    b123 = {"originating_system": "build123d", "preprocessor_version": "Open CASCADE STEP processor 8.0"}
    fc = {"originating_system": "FreeCAD", "preprocessor_version": "Open CASCADE STEP processor 7.8"}
    assert tool_vs_header("build123d", b123) == "consistent"
    assert tool_vs_header("freecad", b123) == "inconsistent"
    assert tool_vs_header("openscad", fc) == "consistent"  # OpenSCAD exports through FreeCAD here
    assert tool_vs_header("rhino", {"originating_system": "unknown"}) == "cannot tell"
    assert tool_vs_header("build123d", None) == "cannot tell"

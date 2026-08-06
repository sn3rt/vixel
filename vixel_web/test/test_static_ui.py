from pathlib import Path
import shutil
import subprocess


PAGE = Path(__file__).parents[1] / "vixel_web/static/index.html"


def test_dashboard_is_dynamic_and_uses_gateway_endpoints():
    page = PAGE.read_text()
    assert "/api/v1/sensors/" in page
    assert "/api/v1/known-sensors" in page
    assert "/api/v1/groups/" in page
    assert "/api/v1/captures" in page
    assert "state.sensors" in page
    assert "state.knownSensors" in page
    assert "state.captures" in page
    assert "Unenroll and archive" in page
    assert "front_left" not in page
    assert "web_video_server" not in page
    assert "Capture and save" in page
    assert "Trigger and publish" in page
    assert "/trigger" in page
    assert 'name="trigger_source"' not in page
    assert "PTP synchronized" in page
    assert "Waiting for PTP lock" in page
    assert "locking_member_ids" in page


def test_dashboard_javascript_parses():
    gjs = shutil.which("gjs")
    assert gjs is not None, "gjs is required to validate dashboard JavaScript"
    page = PAGE.read_text()
    script = page.split("<script>", 1)[1].split("</script>", 1)[0]
    result = subprocess.run(
        [gjs, "-c", f"if (false) {{\n{script}\n}}"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_dashboard_reports_initial_and_event_stream_errors():
    page = PAGE.read_text()
    assert "Initial dashboard load failed" in page
    assert "Inventory update failed" in page
    assert "start();" in page


def test_dashboard_uses_bounded_snapshot_polling_for_camera_cards():
    page = PAGE.read_text()
    assert "data-preview-sensor" in page
    assert "maxConcurrent:3" in page
    assert "refreshMs:500" in page
    assert "schedulePreviews();" in page
    assert "If-None-Match" in page
    assert 'src="/api/v1/sensors/${encodeURIComponent(s.sensor_id)}/stream"' not in page

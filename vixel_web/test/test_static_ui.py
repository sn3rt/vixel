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
    assert "Test shot" in page
    assert "Capture and save" not in page
    assert "Trigger and publish" not in page
    assert "'/api/v1/test-shots'" in page
    assert 'name="trigger_source"' not in page
    assert "PTP synchronized" in page
    assert "Waiting for PTP lock" in page
    assert "locking_member_ids" in page


def test_dashboard_javascript_parses():
    node = shutil.which("node")
    page = PAGE.read_text()
    script = page.split("<script>", 1)[1].split("</script>", 1)[0]
    if node is not None:
        result = subprocess.run(
            [node, "--check"],
            check=False,
            capture_output=True,
            input=script,
            text=True,
        )
    else:
        gjs = shutil.which("gjs")
        assert gjs is not None, "nodejs or gjs is required to validate dashboard JavaScript"
        result = subprocess.run(
            [gjs, "-c", "new Function(ARGV[0]);", script],
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


def test_dashboard_cancel_buttons_close_dialogs_without_submitting_forms():
    page = PAGE.read_text()
    assert page.count('type="button" data-dialog-close>Cancel</button>') == 4
    assert "button.closest('dialog')?.close()" in page
    assert "e.submitter?.value==='cancel'" not in page


def test_dashboard_links_previews_to_a_focused_camera_view():
    page = PAGE.read_text()
    assert 'href="/sensors/${encodeURIComponent(s.sensor_id)}"' in page
    assert 'id="sensor-view"' in page
    assert ".detail-layout { display:grid; grid-template-columns:minmax(0,1fr)" in page
    assert ".detail-preview { width:100%; max-height:none" in page
    assert "detailSensorId" in page
    assert "sensorDetail(sensor)" in page
    assert "Showing the last received frame when available." in page


def test_dashboard_hides_manual_modes_and_separates_test_shot_history():
    page = PAGE.read_text()
    assert "controls('sensor'" not in page
    assert "controls('group'" not in page
    assert ">Snapshot</a>" not in page
    assert "Open snapshot" not in page
    assert 'id="capture-sessions"' in page
    assert 'id="test-shots"' in page
    assert "capture_kind==='test_shot'" in page
    assert "Remove from groups first" in page


def test_dashboard_uses_bounded_snapshot_polling_for_camera_cards():
    page = PAGE.read_text()
    assert "data-preview-sensor" in page
    assert "maxConcurrent:3" in page
    assert "refreshMs:500" in page
    assert "schedulePreviews();" in page
    assert "If-None-Match" in page
    assert 'src="/api/v1/sensors/${encodeURIComponent(s.sensor_id)}/stream"' not in page

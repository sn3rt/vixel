import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).parents[2]


def load_example(name):
    path = ROOT / "examples" / f"{name}.py"
    specification = importlib.util.spec_from_file_location(f"vixel_example_{name}", path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def test_http_example_encodes_group_and_posts_json(monkeypatch):
    example = load_example("http_trigger")
    captured = {}

    class Response(io.BytesIO):
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            self.close()

    def urlopen(request, timeout):
        captured["request"] = request
        captured["timeout"] = timeout
        return Response(json.dumps({
            "success": True,
            "capture_id": "software_1",
            "participating_sensor_ids": ["camera_a"],
        }).encode())

    monkeypatch.setattr(example.urllib.request, "urlopen", urlopen)
    result = example.trigger_group(
        "http://127.0.0.1:8080/", "front pair", "request_1", 4.0
    )

    request = captured["request"]
    assert request.full_url.endswith("/groups/front%20pair/trigger")
    assert request.method == "POST"
    assert json.loads(request.data) == {"request_id": "request_1"}
    assert captured["timeout"] == 4.0
    assert result["capture_id"] == "software_1"


def test_http_example_reports_api_failure(monkeypatch):
    example = load_example("http_trigger")

    class Response(io.BytesIO):
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            self.close()

    monkeypatch.setattr(
        example.urllib.request,
        "urlopen",
        lambda *_args, **_kwargs: Response(
            b'{"success":false,"message":"group must be in capture mode"}'
        ),
    )

    try:
        example.trigger_group("http://127.0.0.1:8080", "front")
    except RuntimeError as error:
        assert "capture mode" in str(error)
    else:
        raise AssertionError("failed API response was accepted")


def test_http_example_can_request_server_side_save(monkeypatch):
    example = load_example("http_trigger")
    captured = {}

    class Response(io.BytesIO):
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            self.close()

    def urlopen(request, timeout):
        captured["request"] = request
        captured["timeout"] = timeout
        return Response(json.dumps({
            "success": True,
            "capture_id": "saved_1",
            "directory": "/var/lib/vixel/captures/2026/08/04/saved_1",
            "saved_sensor_ids": ["camera_a"],
        }).encode())

    monkeypatch.setattr(example.urllib.request, "urlopen", urlopen)
    result = example.request_group(
        "http://127.0.0.1:8080/", "front pair", "save", "request_1", 4.0
    )

    request = captured["request"]
    assert request.full_url.endswith("/groups/front%20pair/capture")
    assert request.method == "POST"
    assert json.loads(request.data) == {"request_id": "request_1"}
    assert captured["timeout"] == 4.0
    assert result["directory"].endswith("/saved_1")


def test_ros_example_timestamp_and_output_names_are_deterministic():
    example = load_example("ros_trigger_and_receive")

    assert example.stamp_key(SimpleNamespace(sec=12, nanosec=34)) == (12, 34)
    assert example.safe_name("capture/front 01") == "capture_front_01"
    assert example.safe_name("") == "capture"


def test_example_parsers_have_runnable_defaults():
    http = load_example("http_trigger").parser().parse_args(["front"])
    periodic = load_example("http_trigger_groups_periodic").parser().parse_args(
        ["front", "back"]
    )
    ros_request = load_example("ros_trigger").parser().parse_args(["front"])
    ros = load_example("ros_trigger_and_receive").parser().parse_args(["front"])

    assert http.base_url == "http://127.0.0.1:8080"
    assert http.mode == "publish"
    assert http.timeout == 35.0
    assert periodic.interval == 2.0
    assert periodic.count == 0
    assert periodic.mode == "publish"
    assert ros_request.mode == "publish"
    assert ros_request.timeout == 35.0
    assert ros.output_dir == Path("vixel-triggered-images")
    assert ros.timeout == 35.0


def test_periodic_example_triggers_groups_and_reports_schedule_delta():
    example = load_example("http_trigger_groups_periodic")
    calls = []

    def request(base_url, group_id, request_id, timeout, mode):
        calls.append((base_url, group_id, request_id, timeout, mode))
        nanosec = 100 if group_id == "front" else 130
        return {"scheduled_time": {"sec": 12, "nanosec": nanosec}}

    results = example.trigger_cycle(
        ["front", "back"], "http://127.0.0.1:8080", 5.0, 3,
        "publish", request,
    )
    times = [example.scheduled_ns(result) for result in results.values()]

    assert set(results) == {"front", "back"}
    assert max(times) - min(times) == 30
    assert {call[1] for call in calls} == {"front", "back"}
    assert {call[2] for call in calls} == {"periodic_3_front", "periodic_3_back"}
    assert {call[4] for call in calls} == {"publish"}


def test_periodic_example_can_save_disjoint_groups():
    example = load_example("http_trigger_groups_periodic")
    calls = []

    def request(base_url, group_id, request_id, timeout, mode):
        calls.append((group_id, request_id, mode))
        return {
            "success": True,
            "capture_id": request_id,
            "directory": f"/captures/{request_id}",
            "saved_sensor_ids": [f"camera_{group_id}"],
        }

    results = example.trigger_cycle(
        ["front", "back"], "http://127.0.0.1:8080", 5.0, 4,
        "save", request,
    )

    assert set(results) == {"front", "back"}
    assert {call[2] for call in calls} == {"save"}
    assert results["front"]["directory"] == "/captures/periodic_4_front"


def test_periodic_save_uses_capture_endpoint(monkeypatch):
    example = load_example("http_trigger_groups_periodic")
    captured = {}

    class Response(io.BytesIO):
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            self.close()

    def urlopen(request, timeout):
        captured["url"] = request.full_url
        captured["body"] = json.loads(request.data)
        captured["timeout"] = timeout
        return Response(b'{"success":true,"capture_id":"saved_front"}')

    monkeypatch.setattr(example.urllib.request, "urlopen", urlopen)
    result = example.request_group(
        "http://127.0.0.1:8080/", "front pair", "saved_front", 4.0, "save"
    )

    assert captured["url"].endswith("/groups/front%20pair/capture")
    assert captured["body"] == {"request_id": "saved_front"}
    assert captured["timeout"] == 4.0
    assert result["capture_id"] == "saved_front"


def test_periodic_sequence_uses_server_managed_cadence(monkeypatch):
    example = load_example("http_trigger_groups_periodic")
    captured = {}

    def request(base_url, path, timeout, body=None):
        captured.update(base_url=base_url, path=path, timeout=timeout, body=body)
        return {"accepted": True, "operation_id": "operation_1"}

    monkeypatch.setattr(example, "api_request", request)
    result = example.start_save_sequence(
        "http://127.0.0.1:8080", ["front", "back"], 0.5, 10, "periodic", 5.0
    )

    assert result["operation_id"] == "operation_1"
    assert captured["path"] == "/api/v1/capture-operations/sequence"
    assert captured["body"]["interval_ms"] == 500
    assert captured["body"]["synchronize_groups"] is True


def test_examples_are_linked_from_documentation():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    operations = (ROOT / "docs" / "operations.md").read_text(encoding="utf-8")

    assert "examples/README.md" in readme
    assert "../examples/README.md" in operations

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


def test_ros_example_timestamp_and_output_names_are_deterministic():
    example = load_example("ros_trigger_and_receive")

    assert example.stamp_key(SimpleNamespace(sec=12, nanosec=34)) == (12, 34)
    assert example.safe_name("capture/front 01") == "capture_front_01"
    assert example.safe_name("") == "capture"


def test_example_parsers_have_runnable_defaults():
    http = load_example("http_trigger").parser().parse_args(["front"])
    ros = load_example("ros_trigger_and_receive").parser().parse_args(["front"])

    assert http.base_url == "http://127.0.0.1:8080"
    assert http.timeout == 35.0
    assert ros.output_dir == Path("vixel-triggered-images")
    assert ros.timeout == 35.0


def test_examples_are_linked_from_documentation():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    operations = (ROOT / "docs" / "operations.md").read_text(encoding="utf-8")

    assert "examples/README.md" in readme
    assert "../examples/README.md" in operations

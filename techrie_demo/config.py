# config.py
import os
import json
from pathlib import Path
import datetime as dt
from openai import AzureOpenAI
from rospkg import RosPack

def pkg_root():
    return RosPack().get_path("techrie_demo")

def data_root():
    # 生成物は source tree ではなく ~/.ros 以下へ
    root = os.path.expanduser("~/.ros/techrie_demo")
    os.makedirs(root, exist_ok=True)
    return root

def resolve_paths(date_str=None):
    """
    旧コードとの互換のため、(day_dir, scenes_json, captioned_json) の3タプルを返す。
    ベースは ~/.ros/techrie_demo/object_images/YYYY/MM/DD/ 配下。
    """
    obj_root = os.path.join(data_root(), "object_images")

    d = dt.date.today() if not date_str else dt.date.fromisoformat(date_str)
    y, m, dd = d.strftime("%Y"), d.strftime("%m"), d.strftime("%d")
    day_dir = os.path.join(obj_root, y, m, dd)
    os.makedirs(day_dir, exist_ok=True)

    scenes_json    = os.path.join(day_dir, f"{d.isoformat()}.json")
    captioned_json = os.path.join(day_dir, f"{d.isoformat()}_captioned.json")
    return day_dir, scenes_json, captioned_json

def resolve_paths_full(date_str=None):
    obj_root = os.path.join(data_root(), "object_images")

    d = dt.date.today() if not date_str else dt.date.fromisoformat(date_str)
    y, m, dd = d.strftime("%Y"), d.strftime("%m"), d.strftime("%d")
    day_dir = os.path.join(obj_root, y, m, dd)
    os.makedirs(day_dir, exist_ok=True)

    return {
        "base": data_root(),
        "day_dir": day_dir,
        "scenes_json": os.path.join(day_dir, f"{d.isoformat()}.json"),
        "captioned_json": os.path.join(day_dir, f"{d.isoformat()}_captioned.json"),
        "diary_json": os.path.join(day_dir, "diary.json"),
        "diary_image": os.path.join(day_dir, "diary_image.png"),
        "diary_combined": os.path.join(day_dir, "diary_combined.png"),
    }

def _load_yaml_config():
    # local config (ignored by git)
    cfg_path = Path(pkg_root()) / "config" / "gpt_api.yaml"
    if not cfg_path.exists():
        return {}

    try:
        import yaml
    except ImportError:
        return {}

    with open(cfg_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}

def _pick(*vals):
    for v in vals:
        if v not in (None, ""):
            return v
    return None

def _text_cfg():
    y = _load_yaml_config().get("text", {})
    return {
        "api_key": _pick(os.getenv("TECHRIE_TEXT_API_KEY"), y.get("api_key")),
        "api_version": _pick(os.getenv("TECHRIE_TEXT_API_VERSION"), y.get("api_version"), "2024-12-01-preview"),
        "base_url": _pick(os.getenv("TECHRIE_TEXT_BASE_URL"), y.get("base_url")),
    }

def _image_cfg():
    y = _load_yaml_config().get("image", {})
    return {
        "api_key": _pick(os.getenv("TECHRIE_IMAGE_API_KEY"), y.get("api_key")),
        "api_version": _pick(os.getenv("TECHRIE_IMAGE_API_VERSION"), y.get("api_version"), "2024-12-01-preview"),
        "azure_endpoint": _pick(os.getenv("TECHRIE_IMAGE_AZURE_ENDPOINT"), y.get("azure_endpoint")),
    }

def get_chat_client():
    c = _text_cfg()
    if not c["api_key"] or not c["base_url"]:
        raise RuntimeError("Azure OpenAI text config is missing. Set config/gpt_api.yaml or TECHRIE_TEXT_* env vars.")
    return AzureOpenAI(
        api_key=c["api_key"],
        api_version=c["api_version"],
        base_url=c["base_url"],
    )

def get_image_client():
    c = _image_cfg()
    if not c["api_key"] or not c["azure_endpoint"]:
        raise RuntimeError("Azure OpenAI image config is missing. Set config/gpt_api.yaml or TECHRIE_IMAGE_* env vars.")
    return AzureOpenAI(
        api_key=c["api_key"],
        api_version=c["api_version"],
        azure_endpoint=c["azure_endpoint"],
    )

# 既存コードとの互換のため（遅延初期化）
class _LazyClient:
    def __init__(self, kind):
        self.kind = kind
        self._client = None

    def _get(self):
        if self._client is None:
            self._client = get_chat_client() if self.kind == "text" else get_image_client()
        return self._client

    def __getattr__(self, name):
        return getattr(self._get(), name)

client = _LazyClient("text")
image_client = _LazyClient("image")

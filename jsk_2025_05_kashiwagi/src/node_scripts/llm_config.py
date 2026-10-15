
#!/usr/bin/env python3
import os
import dspy


def create_lm(model, temperature=1.0, max_tokens=100, **kwargs):
    params = {
        "model": model,
        "temperature": temperature,
        "max_tokens": max_tokens,
        "cache": False,
    }

    if model.startswith("azure/"):
        params.update({
            "api_key": os.environ["AZURE_OPENAI_KEY"],
            "api_base": os.environ["AZURE_OPENAI_ENDPOINT"],
            "api_version": "2024-12-01-preview",
        })

    elif model.startswith("ollama_chat/"):
        params.update({
            "api_key": "",
            "api_base": os.getenv(
                "LOCAL_LLM_URL", "http://localhost:11434"
            ),
        })

    params.update(kwargs)

    return dspy.LM(**params)

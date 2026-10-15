
import os
from openai import OpenAI

endpoint = os.getenv("AZURE_OPENAI_ENDPOINT_IORY")
api_key = os.getenv("AZURE_OPENAI_KEY_IORY")
deployment_name = "gpt-5.4"
client = OpenAI(
    base_url=endpoint,
    api_key=api_key,
)

response = client.responses.create(
    model=deployment_name,
    input="日本語で一言だけ返して。返答は「こんにちは」にして。",
)

print(response.output_text)

from openai import OpenAI
from azure.identity import DefaultAzureCredential, get_bearer_token_provider
import os

endpoint = os.getenv("AZURE_OPENAI_ENDPOINT_IORY")
deployment_name = "gpt-5.4"

client = OpenAI(
    base_url=endpoint,
    api_key=os.getenv("AZURE_OPENAI_KEY_IORY")
)

response = client.responses.create(
    model=deployment_name,
    input="What is the capital of France?",
)

print(f"answer: {response.output[0]}")

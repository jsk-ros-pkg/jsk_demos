# step4_generate_image.py
import base64
import json
from config import image_client


def generate_diary_image(diary_text, paths,profile):
    prompt_image = (
        f"Children's book illustration. Show a robot named {profile['name']} with this appearance: {profile['appearance']}.\n"
        f"Use the following diary to inspire the image. Do not include text in the image.\n"
        f"{diary_text}"
    )

    res = image_client.images.generate(
        model="gpt-image-1",
        prompt=prompt_image,
        n=1,
        quality="medium"
    )

    image_b64 = json.loads(res.model_dump_json())["data"][0]["b64_json"]
    with open(paths["output_image_path"], "wb") as f:
        f.write(base64.b64decode(image_b64))
    print(f"🖼️ 絵日記の画像を保存: {paths['output_image_path']}")

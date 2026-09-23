from transformers import AutoModelForCausalLM
from PIL import Image

model = AutoModelForCausalLM.from_pretrained(
    "vikhyatk/moondream2", revision="2025-04-14", trust_remote_code=True,
)

img = Image.open("frame.png")
print("image size:", img.size)

for query in ["the red box", "the green box", "the blue box", "the white platform"]:
    result = model.point(img, query)
    print(f"{query}: {result}")

from transformers import AutoModelForCausalLM
from PIL import Image

model = AutoModelForCausalLM.from_pretrained(
    "vikhyatk/moondream2", revision="2025-04-14", trust_remote_code=True,
)
img = Image.open("test.jpg")
print("image size:", img.size)

print("\ncaption:")
print(model.caption(img, length="short")["caption"])

print("\npoint 'object':")
print(model.point(img, "object"))

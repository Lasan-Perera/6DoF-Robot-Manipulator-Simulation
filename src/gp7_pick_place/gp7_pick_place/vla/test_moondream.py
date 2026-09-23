from transformers import AutoModelForCausalLM
from PIL import Image
import time

print("loading model (first run downloads weights, may take a few minutes)...")
model = AutoModelForCausalLM.from_pretrained(
    "vikhyatk/moondream2",
    revision="2025-04-14",
    trust_remote_code=True,
)
print("loaded.")

img = Image.open("test.jpg")

start = time.time()
result = model.point(img, "person")
print(f"took {time.time()-start:.1f}s")
print(result)

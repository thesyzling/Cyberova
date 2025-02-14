from PIL import Image
import pytesseract

img=Image.open("py.18.png")
text=pytesseract.image_to_string(img,lang="eng")
print(text)

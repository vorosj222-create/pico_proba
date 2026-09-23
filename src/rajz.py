from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.utils import ImageReader

img_path = "/mnt/data/017a7248-95e2-4a51-8f1a-541be0177518.png"
pdf_path = "/mnt/data/Robotarm_Heligears_sketch.pdf"

page_w, page_h = landscape(A4)
c = canvas.Canvas(pdf_path, pagesize=(page_w, page_h))

img = ImageReader(img_path)
iw, ih = img.getSize()

# Fit the screenshot to the page while preserving its aspect ratio.
scale = min(page_w / iw, page_h / ih)
dw, dh = iw * scale, ih * scale
x = (page_w - dw) / 2
y = (page_h - dh) / 2

c.drawImage(img, x, y, width=dw, height=dh)
c.showPage()
c.save()

print(pdf_path)

import pdfplumber

with pdfplumber.open('MKS-SERVO42C-User-Manual-V1.1.2.pdf') as pdf:
    page = pdf.pages[25]  # page 26 (0-based)
    text = page.extract_text()
    print(text)
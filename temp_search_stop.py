import pdfplumber

with pdfplumber.open('MKS-SERVO42C-User-Manual-V1.1.2.pdf') as pdf:
    for i, page in enumerate(pdf.pages):
        text = page.extract_text()
        if 'stop' in text.lower():
            print(f"Page {i+1}:")
            print(text)
            print("\n" + "="*50 + "\n")
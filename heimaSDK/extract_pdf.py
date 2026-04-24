from pypdf import PdfReader

try:
    reader = PdfReader("/Users/shenghuang/Downloads/heima/heimaFrameWork/heimaSDK/Ethercat使用说明教程.pdf")
    text = ""
    for page in reader.pages:
        text += page.extract_text()
    with open("/Users/shenghuang/Downloads/heima/heimaFrameWork/heimaSDK/Ethercat_doc.txt", "w") as f:
        f.write(text)
    print("PDF extraction successful.")
except Exception as e:
    print(f"Error: {e}")

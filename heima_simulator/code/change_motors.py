import re

file_path = '/Users/shenghuang/Downloads/heima/heimaFrameWork/heima_simulator/code/main.cpp'
with open(file_path, 'r') as f:
    text = f.read()

# Replace hardcoded 15 with 13 in arrays
text = re.sub(r'\[15\]', '[13]', text)
text = re.sub(r'15', '13', text)

with open(file_path, 'w') as f:
    f.write(text)

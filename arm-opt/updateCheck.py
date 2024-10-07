# Checks for updated solver files
import os
import hashlib

def check():
    files = [file for file in os.listdir(".") if file.endswith(".py")]
    textbuffer = ""
    for i in files:
        textbuffer += ''.join(open(i, 'r').readlines())
    result = int(hashlib.sha1(textbuffer.encode()).hexdigest(), 16)
    try:
        old_result = open("paths/.last_result", 'r').readlines()[0].strip("\n")
    except:
        old_result = ""
    if old_result == str(result):
        return False
    else:
        out = open("paths/.last_result", 'w')
        out.write(str(result))
        out.close()
        return True

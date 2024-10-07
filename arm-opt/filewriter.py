import json

def importData(fileName):
    f = open(fileName)
    data = json.load(f)
    f.close()
    return data

def saveData(fileName, data):
    f = open(fileName, "w")
    formattedData = json.dumps(data, sort_keys=True, indent=4)
    f.write(formattedData)
    f.close()
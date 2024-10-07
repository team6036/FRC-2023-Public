import filewriter

pi = 3.14159265

presets = {}


def reload():
    global presets, pi
    presets = filewriter.importData("arm_presets.json")

    for position in presets:
        presets[position][1] = -(2 * pi - presets[position][1])



reload()
